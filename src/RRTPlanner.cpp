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
    vehicleModel = make_shared<VehicleModel> (VehicleModel(move(vehicleParam)));
    mapHandler = make_shared<MapHandler> (MapHandler(move(mapParam), vehicleModel));
    localRRT = unique_ptr<SearchTree>(new SearchTree(make_shared<SS_VECTOR> (SS_VECTOR()), LOCAL_RRT));
    globalRRT = unique_ptr<SearchTree>(new SearchTree(make_shared<SS_VECTOR> (SS_VECTOR()), GLOBAL_RRT));

    Control::setParameters(move(controlParam));

    // Load ROS parameters
    loadParameters();

    // Subscribe to map
    ROS_INFO_STREAM("" << nodeName << " Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &(*(mapHandler.get())));
    blueTrackBoundarySubscriber = nh.subscribe("/blueTrackBoundary", 1, &MapHandler::blueTrackBoundaryCallback, &(*(mapHandler.get())));
    yellowTrackBoundarySubscriber = nh.subscribe("/yellowTrackBoundary", 1, &MapHandler::yellowTrackBoundaryCallback, &(*(mapHandler.get())));
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &(*(vehicleModel)));
    SLAMStatusSubscriber = nh.subscribe("/slam_status", 1, &MapHandler::SLAMStatusCallback, &(*(mapHandler.get())));
    odometrySubscriber = nh.subscribe("/odometry/velocity", 1, &VehicleModel::velocityCallback, &(*(vehicleModel)));
    localRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_local_viz", 10);
    globalRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_global_viz", 10);
    commonPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_common_viz", 10);

    timer = nh.createWallTimer(ros::WallDuration(genParam->timerPeriod), &RRTPlanner::timerCallback, this);
    
    ros::spin();
}


void RRTPlanner::stateMachine()
{
    switch(state)
    {
        case NOMAP:
            if (mapHandler->getState() != EMPTY) state = LOCALPLANNING;
            break;
        case LOCALPLANNING:
            planLocalRRT();
            if (mapHandler->isLoopClosed() && handleActualPath())
            {
                state = WAITFORGLOBAL;
            }
            break;
        case WAITFORGLOBAL:
            planLocalRRT();
            planGlobalRRT();
            if(globalRRT->pathFound) state = GLOBALPLANNING;
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
    shared_ptr<SS_VECTOR> randState;
    shared_ptr<SS_VECTOR> newState;
    shared_ptr<PATH_TYPE> trajectory;

    // Get random state
    randState = mapHandler->getRandomState(rrt->getBestPath(), rrt->param);

    // Get nearest node
    shared_ptr<SearchTreeNode> nearest = rrt->getNearest(randState);

    // Simulate movement towards new state
    trajectory = vehicleModel->simulateToTarget(nearest->getState(), randState, rrt->param);

    // Check for offCourse
    offCourse = mapHandler->isOffCourse(trajectory);

    // Add node
    if ((!offCourse) && (trajectory->size() > 1))
    {
        // Check if new node would be duplicate
        newState = trajectory->back();
        alreadyInTree = rrt->alreadyInTree(newState);

        if(!alreadyInTree)
        {
            newNode = rrt->addChild(nearest, newState, trajectory->cost(rrt->param));
        }
    }
    else
    {
        newNode = NULL;
    }

    return newNode;
}

bool RRTPlanner::rewire(unique_ptr<SearchTree>& rrt, shared_ptr<SearchTreeNode> newNode)
{
    shared_ptr<PATH_TYPE> trajectory;
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> nearbyNodes = rrt->getNearby(newNode);
    vector<shared_ptr<SearchTreeNode>>::iterator it;
    float newNodeCost = rrt->getAbsCost(newNode);
    bool offCourse;

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = vehicleModel->simulateToTarget(newNode->getState(), (*it)->getState(), rrt->param, rrt->param->rewireTime);
        offCourse = mapHandler->isOffCourse(trajectory);
        if ((trajectory->size() > 1) && !offCourse)
        {
            float distError = trajectory->back()->getDistOriented(*(*it)->getState(), rrt->param);
            // Check if new path leads close to new state
            if ( distError < rrt->param->minDeviation)
            {
                float segmentCost = trajectory->cost(rrt->param);
                float childCost = rrt->getAbsCost(*it);
                float timeError = distError / trajectory->back()->v();
                // Compare costs  if it is worth rewiring
                if ((newNodeCost + segmentCost + timeError) < childCost)
                {
                    //Rewire if it reduces cost
                    rrt->rewire(*it,newNode);
                    (*it)->changeSegmentCost(segmentCost);
                }
                else if ((newNodeCost - globalRRT->param->minCost) > childCost)
                {
                    // If newNodeCost is significantly higher than childCost, a loop might be created
                    // Rewiring would create a loop with multiple problems
                    // Instead of making a loop, mark it in the SearchTree
                    bool isLoop = rrt->addLoop(newNode, *it, segmentCost);

                    // If a loop is not created, rewire 
                    if(!isLoop)
                    {
                        rrt->rewire(*it,newNode);
                        (*it)->changeSegmentCost(segmentCost);
                    }
                }
            }
        }
    }
    return false;
}


void RRTPlanner::optimizeTriangles(unique_ptr<SearchTree>& rrt)
{
    for(int i = 0; i < rrt->param->triangleIterations; i++)
    {
        shared_ptr<SearchTreeNode> node = rrt->getRandomNode();
        optimizeTriangle(rrt, node);
    }
}

void RRTPlanner::optimizeTriangle(unique_ptr<SearchTree>& rrt, shared_ptr<SearchTreeNode> node)
{
    shared_ptr<SearchTreeNode> parent, parentParent;
    shared_ptr<PATH_TYPE> trajectory;
    bool isClose, isLowerCost, offCourse;
    float segmentCost;

    parent = node->getParent();
    if(parent == NULL) return;
    parentParent = parent->getParent();
    if(parentParent == NULL) return;

    trajectory = vehicleModel->simulateToTarget(parentParent->getState(), node->getState(), rrt->param, rrt->param->rewireTime);
    if(trajectory->size() > 0)
    {
        segmentCost = trajectory->cost(rrt->param);
        shared_ptr<StateSpace2D> s = node->getState();
        float error = trajectory->back()->getDistOriented(*s, rrt->param);
        isClose =  error < rrt->param->minDeviation;
        isLowerCost = segmentCost < parent->getSegmentCost() + node->getSegmentCost() + rrt->param->minDeviation / (parentParent->getState()->v() *2);
        offCourse = mapHandler->isOffCourse(trajectory);

        if(isClose && isLowerCost && !offCourse)
        {
            rrt->rewire(node, parentParent);
            node->changeSegmentCost(segmentCost);
            if(parent->getChildren()->size() == 0)
            {
                rrt->remove(parent);
            }

        }
    }

}

void RRTPlanner::planLocalRRT(void)
{
    shared_ptr<SS_VECTOR> pose = vehicleModel->getCurrentPose();
    localRRT->init(pose);
    int iteration = 0;
    shared_ptr<SS_VECTOR> goalState = mapHandler->getGoalState();

    while((!localRRT->maxNumOfNodesReached()) && (iteration <= localRRT->param->iterations))
    {
        shared_ptr<SearchTreeNode> node = extend(localRRT);
        if (node != NULL)
        {   
            bool rewired = rewire(localRRT, node);
            bool isClose = node->getState()->getDistEuclidean(*goalState) < localRRT->param->goalRadius;
            if ((localRRT->pathFound && rewired) || isClose)
            {
                localRRT->updatePath(localRRT->traceBackToRoot(localRRT->getNearest(goalState)));
            }

            if(isClose)
            {
                localRRT->pathFound = true;
            }
        }

        optimizeTriangles(localRRT);

        iteration++;
    }

}

void RRTPlanner::planGlobalRRT(void)
{
    int iteration = 0;
    bool changed = false;
    shared_ptr<SS_VECTOR> goalState = mapHandler->getGoalState();
    while((!globalRRT->maxNumOfNodesReached()) && (iteration <= globalRRT->param->iterations))
    {
        shared_ptr<SearchTreeNode> node = extend(globalRRT);
        if (node != NULL)
        {   
            bool rewired = rewire(globalRRT, node);
            if(rewired) changed = true;

            //float goalDist = node->getState()->getDistToTarget(*globalRRT->getRoot(), globalRRT->param);
            //if(goalDist < globalRRT->param->goalRadius) globalRRT->pathFound = true;
        }

        optimizeTriangles(globalRRT);
        
        iteration++;
    }

    globalRRT->manageLoops();
}

bool RRTPlanner::handleActualPath(void)
{
    shared_ptr<PATH_TYPE> actualPath = vehicleModel->getActualPath();
    float fullCost = actualPath->cost(globalRRT->param);
    if (fullCost < globalRRT->param->minCost) return false;

    shared_ptr<PATH_TYPE> loop = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    shared_ptr<SS_VECTOR> currentPose = actualPath->back();
    PATH_TYPE::iterator it;
    bool isLoop = false;
    float cost = 0;
    PATH_TYPE segment;
    segment.push_back(actualPath->front());

    float distStep = globalRRT->param->simulationTimeStep * currentPose->v();

    for(it = actualPath->begin()+1; it != actualPath->end(); it++)
    {
        if ((currentPose->getDistToTarget(**it, globalRRT->param) < distStep) &&
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
    visualize();
    ros::Duration runTime = ros::Time::now() - startTime;
    if(runTime.toSec() > genParam->timerPeriod)
    {
         ROS_WARN_STREAM("[" << ros::this_node::getName() << "] Runtime high: " << ((int) (runTime.toSec()*1000)) << " ms");
    }
}

void RRTPlanner::visualize(void)
{
    localRRT->markerArray.markers.clear();
    localRRT->visualize();
    localRRT->markerPublisher.publish(localRRT->markerArray);

    globalRRT->markerArray.markers.clear();
    globalRRT->visualize();
    globalRRT->markerPublisher.publish(globalRRT->markerArray);

    commonMArray.markers.clear();
    mapHandler->visualize(&commonMArray);
    vehicleModel->visualize(&commonMArray); //TODO
    commonPublisher.publish(commonMArray);
}


int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);

    return 0;
}


void RRTPlanner::loadParameter(const string& topic, float& parameter, const float defaultValue)
{
    if (!ros::param::get("/rrt_planner/" + topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  std::to_string(parameter)); 
}

void RRTPlanner::loadParameter(const string& topic, int& parameter, const int defaultValue)
{
    if (!ros::param::get("/rrt_planner/" + topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  std::to_string(parameter)); 
}

void RRTPlanner::loadParameter(const string& topic, string& parameter, const string defaultValue)
{
    if (!ros::param::get("/rrt_planner/" + topic, parameter))
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

    loadParameter("/LOCAL/maxNumOfNodes", localRRT->param->maxNumOfNodes, 1000);
    loadParameter("/GLOBAL/maxNumOfNodes", globalRRT->param->maxNumOfNodes, 1000);

    loadParameter("/LOCAL/minCost", localRRT->param->minCost, 0.0f);
    loadParameter("/GLOBAL/minCost", globalRRT->param->minCost, 3.0f);

    loadParameter("/LOCAL/minDeviation", localRRT->param->minDeviation, 0.1f);
    loadParameter("/GLOBAL/minDeviation", globalRRT->param->minDeviation, 0.1f);

    loadParameter("/LOCAL/minDeviationDist", localRRT->param->minDeviationDist, 0.05f);
    loadParameter("/GLOBAL/minDeviationDist", globalRRT->param->minDeviationDist, 0.05f);

    loadParameter("/LOCAL/psiWeight", localRRT->param->psiWeight, 5000.0f);
    loadParameter("/GLOBAL/psiWeight", globalRRT->param->psiWeight, 5000.0f);

    loadParameter("/LOCAL/resolution", localRRT->param->resolution, 0.05f);
    loadParameter("/GLOBAL/resolution", globalRRT->param->resolution, 0.05f);

    loadParameter("/LOCAL/rewireTime", localRRT->param->rewireTime, 3.0f);
    loadParameter("/GLOBAL/rewireTime", globalRRT->param->rewireTime, 3.0f);

    loadParameter("/LOCAL/sampleRange", localRRT->param->sampleRange, 3);
    loadParameter("/GLOBAL/sampleRange", globalRRT->param->sampleRange, 3);

    loadParameter("/LOCAL/simIterations", localRRT->param->simIterations, 1);
    loadParameter("/GLOBAL/simIterations", globalRRT->param->simIterations, 1);

    loadParameter("/LOCAL/simulationTimeStep", localRRT->param->simulationTimeStep, 0.2f);
    loadParameter("/GLOBAL/simulationTimeStep", globalRRT->param->simulationTimeStep, 0.2f);

    loadParameter("/LOCAL/thetaWeight", localRRT->param->thetaWeight, 50.0f);
    loadParameter("/GLOBAL/thetaWeight", globalRRT->param->thetaWeight, 50.0f);

    loadParameter("/LOCAL/triangleIterations", localRRT->param->triangleIterations, 1.0f);
    loadParameter("/GLOBAL/triangleIterations", globalRRT->param->triangleIterations, 1.0f);

    loadParameter("/VEHICLE/maxdDelta", vehicleModel->getParameters()->maxdDelta, 0.1f);
    loadParameter("/VEHICLE/maxDelta", vehicleModel->getParameters()->maxDelta, 0.38f);
    loadParameter("/VEHICLE/maxLatAccel", vehicleModel->getParameters()->maxLatAccel, 10.0f);
    loadParameter("/VEHICLE/maxLongAccel", vehicleModel->getParameters()->maxLongAccel, 5.0f);
    loadParameter("/VEHICLE/track", vehicleModel->getParameters()->track, 1.2f);
    loadParameter("/VEHICLE/wheelBase", vehicleModel->getParameters()->wheelBase, 1.54f); 

    loadParameter("/MAP/collisionRange", mapHandler->getParameters()->collisionRange, 6.0f);
    loadParameter("/MAP/goalHorizon", mapHandler->getParameters()->goalHorizon, 15.0f);
    loadParameter("/MAP/maxConeDist", mapHandler->getParameters()->maxConeDist, 6.0f);
    loadParameter("/MAP/maxGap", mapHandler->getParameters()->maxGap, 1.5f);

    loadParameter("/CONTROL/k", Control::getParameters()->k, 15.0f);

    // Choose simulation type
    std::string simType;
    loadParameter("/VEHICLE/simType", simType, "HOLONOMIC");
    if (simType == "HOLONOMIC") vehicleModel->getParameters()->simType = HOLONOMIC;
    else if (simType == "HOLONOMIC_CONSTRAINED") vehicleModel->getParameters()->simType = HOLONOMIC_CONSTRAINED;
    else if (simType == "BICYCLE_SIMPLE") vehicleModel->getParameters()->simType = BICYCLE_SIMPLE;
    else if (simType == "BICYCLE") vehicleModel->getParameters()->simType = BICYCLE;
}