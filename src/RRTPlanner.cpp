#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;
    nodeName = "[" + ros::this_node::getName() + "]";

    state = NOMAP;

    // Init parameter structs
    genParam = unique_ptr<GENERAL_PARAMETERS> (new GENERAL_PARAMETERS);
    unique_ptr<VEHICLE_PARAMETERS> vehicleParam = unique_ptr<VEHICLE_PARAMETERS> (new VEHICLE_PARAMETERS);
    unique_ptr<MAP_PARAMETERS> mapParam = unique_ptr<MAP_PARAMETERS> (new MAP_PARAMETERS);
    unique_ptr<CONTROL_PARAMETERS> controlParam = unique_ptr<CONTROL_PARAMETERS> (new CONTROL_PARAMETERS);

    // Init objects with zero parameters
    localRRT = unique_ptr<SearchTree>(new SearchTree(&vehicleModel, SS_VECTOR(0.0, 0.0, 0.0), LOCAL_RRT));
    globalRRT = unique_ptr<SearchTree>(new SearchTree(&vehicleModel, SS_VECTOR(0.0, 0.0, 0.0), GLOBAL_RRT));

    vehicleModel = VehicleModel(move(vehicleParam));
    mapHandler = MapHandler(move(mapParam), &vehicleModel);
    Control::setParameters(move(controlParam));

    // Load ROS parameters
    loadParameters();

    // Subscribe to map
    ROS_INFO_STREAM("" << nodeName << " Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    SLAMStatusSubscriber = nh.subscribe("/slam_status", 1, &MapHandler::SLAMStatusCallback, &mapHandler);
    odometrySubscriber = nh.subscribe("/odometry/velocity", 1, &VehicleModel::velocityCallback, &vehicleModel);
    localRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_local_viz", 10);
    globalRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_global_viz", 10);

    timer = nh.createWallTimer(ros::WallDuration(genParam->timerPeriod), &RRTPlanner::timerCallback, this);
    
    ros::spin();
}


void RRTPlanner::stateMachine()
{
    switch(state)
    {
        case NOMAP:
            if (mapHandler.hasMap()) state = LOCALPLANNING;
            break;
        case LOCALPLANNING:
            planLocalRRT();
            if (mapHandler.isLoopClosed() && handleActualPath())
            {
                state = WAITFORGLOBAL;
            }
            break;
        case WAITFORGLOBAL:
            planLocalRRT();
            planGlobalRRT();
            if(globalRRT->pathClosed) state = GLOBALPLANNING;
            break;
        case GLOBALPLANNING:
            planGlobalRRT();
            break;
    }
}


shared_ptr<SearchTreeNode> RRTPlanner::extend(unique_ptr<SearchTree>& rrt)
{
    shared_ptr<SearchTreeNode> newNode;
    bool offCourse, alreadyInTree;
    SS_VECTOR* randState;
    SS_VECTOR newState;
    shared_ptr<PATH_TYPE> trajectory;

    // Get random state
    randState = mapHandler.getRandomState(rrt->bestPath, rrt->param);

    // Get nearest node
    shared_ptr<SearchTreeNode> nearest = rrt->getNearest(randState);

    // Simulate movement towards new state
    trajectory = vehicleModel.simulateToTarget(nearest->getState(), randState, rrt->param);

    // Check for offCourse
    offCourse = mapHandler.isOffCourse(trajectory, rrt->param);

    // Add node
    if ((!offCourse) && (trajectory->size() > 0))
    {
        // Check if new node would be duplicate
        newState = trajectory->back();
        alreadyInTree = rrt->alreadyInTree(&newState);

        if(!alreadyInTree)
        {
            newNode = rrt->addChild(nearest, newState, trajectory->cost(rrt->param));
        }
    }
    else
    {
        newNode = NULL;
    }
    delete randState;

    return newNode;
}

bool RRTPlanner::rewire(unique_ptr<SearchTree>& rrt, shared_ptr<SearchTreeNode> newNode)
{
    shared_ptr<PATH_TYPE> trajectory;
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> nearbyNodes = rrt->getNearby(newNode);
    vector<shared_ptr<SearchTreeNode>>::iterator it;
    float newNodeCost = rrt->getAbsCost(newNode);

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = vehicleModel.simulateToTarget(newNode->getState(), (*it)->getState(), rrt->param);
        if (trajectory->size() > 0)
        {
            // Check if new path leads close to new state
            if (trajectory->back().getDistOriented((*it)->getState(), rrt->param) < rrt->param->minDeviation)
            {
                float segmentCost = trajectory->cost(rrt->param);
                float childCost = rrt->getAbsCost(*it);
                // Compare costs. If the childCost is significantly
                if (((newNodeCost + segmentCost) < childCost) || ((newNodeCost - rrt->param->minCost) > childCost))
                {
                    //Rewire if it reduces cost
                    rrt->rewire(*it,newNode);
                    (*it)->changeSegmentCost(segmentCost);
                    
                    // If a node with lower cost is rewired to another, this causes a loop and has to be marked
                    (*it)->setRoot(newNodeCost > childCost);
                }
            }
        }
    }
    return false;
}

void RRTPlanner::planLocalRRT(void)
{
    SS_VECTOR* pose = vehicleModel.getCurrentPose();
    localRRT->init(pose);
    localRRT->pathFound = false;
    int iteration = 0;
    SS_VECTOR goalState = mapHandler.getGoalState();

    // TODO
    while((!localRRT->maxNumOfNodesReached()) && (iteration <= localRRT->param->iterations))
    {
        shared_ptr<SearchTreeNode> node = extend(localRRT);
        if (node != NULL)
        {   
            rewire(localRRT, node);
            float goalDist = node->getState()->getDistEuclidean(&goalState);
            if(goalDist < localRRT->param->goalRadius) localRRT->pathFound = true;
        }
        iteration++;
    }

    if (localRRT->pathFound)
    {
        localRRT->bestPath = localRRT->traceBackToRoot(&goalState);
        localRRT->pathLength = localRRT->bestPath->getDistanceCost();
        localRRT->pathTime = localRRT->bestPath->getTimeCost();
    }

    static bool isSaved = false;
    if(!isSaved)
    {
        {
            std::ofstream f( "tree.xml" );
            cereal::XMLOutputArchive archive( f );

            archive( (*localRRT) );
            archive.serializeDeferments();
        }
        isSaved = true;
    }
}

void RRTPlanner::planGlobalRRT(void)
{
    int iteration = 0;
    bool changed = false;
    SS_VECTOR goalState = mapHandler.getGoalState();
    while((!globalRRT->maxNumOfNodesReached()) && (iteration <= globalRRT->param->iterations))
    {
        shared_ptr<SearchTreeNode> node = extend(globalRRT);
        if (node != NULL)
        {   
            bool rewired = rewire(globalRRT, node);
            if(rewired) changed = true;

            float goalDist = node->getState()->getDistToTarget(globalRRT->getRoot(), globalRRT->param);
            if(goalDist < globalRRT->param->goalRadius) globalRRT->pathFound = true;
        }
        iteration++;
    }

    if (globalRRT->pathFound)
    {
        globalRRT->bestPath = globalRRT->traceBackToRoot(globalRRT->getRoot());
        globalRRT->pathLength = globalRRT->bestPath->getDistanceCost();
        globalRRT->pathTime = globalRRT->bestPath->getTimeCost();
    }
}

bool RRTPlanner::handleActualPath(void)
{
    shared_ptr<PATH_TYPE> actualPath = vehicleModel.getActualPath();
    float fullCost = actualPath->cost(globalRRT->param);
    if (fullCost < globalRRT->param->minCost) return false;

    shared_ptr<PATH_TYPE> loop = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    SS_VECTOR currentPose = actualPath->back();
    PATH_TYPE::iterator it;
    bool isLoop = false;
    float cost = 0;
    PATH_TYPE segment;
    segment.push_back(actualPath->front());

    float distStep = globalRRT->param->simulationTimeStep * currentPose.v();

    for(it = actualPath->begin()+1; it != actualPath->end(); it++)
    {
        if ((currentPose.getDistToTarget(&(*it), globalRRT->param) < distStep) &&
            (cost < (fullCost - 3* distStep)))
        {
            isLoop = true;
        }
        if (isLoop)
        {
            loop->push_back(*it);
        }

        // Calculate cost
        segment.push_back(*it);
        cost += segment.cost(globalRRT->param);

        // Reset segment
        segment.erase(segment.begin());
    }

    if(isLoop)
    {
        globalRRT->init(loop);
    }

    return isLoop;
}

void RRTPlanner::timerCallback(const ros::WallTimerEvent &event)
{
    ros::Time startTime = ros::Time::now();
    stateMachine();
    visualize(localRRT);
    visualize(globalRRT);
    ros::Duration runTime = ros::Time::now() - startTime;
    if(runTime.toSec() > genParam->timerPeriod)
    {
         ROS_WARN_STREAM("[" << ros::this_node::getName() << "] Runtime high: " << ((int) (runTime.toSec()*1000)) << " ms");
    }
}

void RRTPlanner::visualize(unique_ptr<SearchTree>& rrt)
{
    rrt->markerArray.markers.clear();
    rrt->visualize();
    mapHandler.visualizePoints(&rrt->markerArray);
    vehicleModel.visualize(&rrt->markerArray); //TODO
    rrt->markerPublisher.publish(rrt->markerArray);
}


int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);

    return 0;
}


void RRTPlanner::loadParameter(const string& topic, float& parameter, const float defaultValue)
{
    if (!ros::param::get(topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  std::to_string(parameter)); 
}

void RRTPlanner::loadParameter(const string& topic, int& parameter, const int defaultValue)
{
    if (!ros::param::get(topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  std::to_string(parameter)); 
}

void RRTPlanner::loadParameter(const string& topic, string& parameter, const string defaultValue)
{
    if (!ros::param::get(topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  parameter);     
}

void RRTPlanner::loadParameters(void)
{
    ROS_INFO_STREAM(nodeName << " LOADING PARAMETERS");
    loadParameter("/GENERAL/timerPeriod", genParam->timerPeriod, 0.1f);

    loadParameter("/LOCAL/collisionRange", localRRT->param->collisionRange, 6.0f);
    loadParameter("/GLOBAL/collisionRange", globalRRT->param->collisionRange, 6.0f);

    // Choose distance calculation type
    std::string costType;
    loadParameter("/LOCAL/costType", costType, "DISTANCE");
    if (costType == "DISTANCE") localRRT->param->costType = DISTANCE;
    else if (costType == "TIME") localRRT->param->costType = TIME;

    loadParameter("/GLOBAL/costType", costType, "DISTANCE");
    if (costType == "DISTANCE") globalRRT->param->costType = DISTANCE;
    else if (costType == "TIME") globalRRT->param->costType = TIME;

    loadParameter("/LOCAL/goalBias", localRRT->param->goalBias, 0.2f);
    loadParameter("/GLOBAL/goalBias", globalRRT->param->goalBias, 0.2f);

    loadParameter("/LOCAL/goalRadius", localRRT->param->goalRadius, 1.0f);
    loadParameter("/GLOBAL/goalRadius", globalRRT->param->goalRadius, 1.0f);

    loadParameter("/LOCAL/iterations", localRRT->param->iterations, 500);
    loadParameter("/GLOBAL/iterations", globalRRT->param->iterations, 500);

    loadParameter("/LOCAL/maxConeDist", localRRT->param->maxConeDist, 6.0f);
    loadParameter("/GLOBAL/maxConeDist", globalRRT->param->maxConeDist, 6.0f);

    loadParameter("/LOCAL/maxNumOfNodes", localRRT->param->maxNumOfNodes, 1000);
    loadParameter("/GLOBAL/maxNumOfNodes", globalRRT->param->maxNumOfNodes, 1000);

    loadParameter("/LOCAL/maxVelocity", localRRT->param->maxVelocity, 10.0f);
    loadParameter("/GLOBAL/maxVelocity", globalRRT->param->maxVelocity, 10.0f);

    loadParameter("/LOCAL/minCost", localRRT->param->minCost, 0.0f);
    loadParameter("/GLOBAL/minCost", globalRRT->param->minCost, 3.0f);

    loadParameter("/LOCAL/minDeviation", localRRT->param->minDeviation, 0.05f);
    loadParameter("/GLOBAL/minDeviation", globalRRT->param->minDeviation, 0.05f);

    loadParameter("/LOCAL/resolution", localRRT->param->resolution, 0.05f);
    loadParameter("/GLOBAL/resolution", globalRRT->param->resolution, 0.05f);

    loadParameter("/LOCAL/rewireRange", localRRT->param->rewireRange, 1.0f);
    loadParameter("/GLOBAL/rewireRange", globalRRT->param->rewireRange, 1.0f);

    loadParameter("/LOCAL/sampleRange", localRRT->param->sampleRange, 3);
    loadParameter("/GLOBAL/sampleRange", globalRRT->param->sampleRange, 3);

    loadParameter("/LOCAL/simulationTimeStep", localRRT->param->simulationTimeStep, 0.1f);
    loadParameter("/GLOBAL/simulationTimeStep", globalRRT->param->simulationTimeStep, 0.1f);

    loadParameter("/LOCAL/thetaWeight", localRRT->param->thetaWeight, 0.1f);
    loadParameter("/GLOBAL/thetaWeight", globalRRT->param->thetaWeight, 0.1f);

    loadParameter("/VEHICLE/maxDelta", vehicleModel.getParameters()->maxDelta, 0.38f);
    loadParameter("/VEHICLE/track", vehicleModel.getParameters()->track, 1.2f);
    loadParameter("/VEHICLE/wheelBase", vehicleModel.getParameters()->wheelBase, 1.54f); 

    loadParameter("/MAP/goalHorizon", mapHandler.getParameters()->goalHorizon, 15.0f);

    loadParameter("/CONTROL/k", Control::getParameters()->k, 15.0f);
    loadParameter("/CONTROL/maxdDelta", Control::getParameters()->maxdDelta, 0.1f);
    loadParameter("/CONTROL/maxLongAccel", Control::getParameters()->maxLongAccel, 5.0f);

    // Choose simulation type
    std::string simType;
    loadParameter("/VEHICLE/simType", simType, "HOLONOMIC");
    if (simType == "HOLONOMIC") vehicleModel.getParameters()->simType = HOLONOMIC;
    else if (simType == "HOLONOMIC_CONSTRAINED") vehicleModel.getParameters()->simType = HOLONOMIC_CONSTRAINED;
    else if (simType == "BICYCLE_SIMPLE") vehicleModel.getParameters()->simType = BICYCLE_SIMPLE;
    else if (simType == "BICYCLE") vehicleModel.getParameters()->simType = BICYCLE;
}