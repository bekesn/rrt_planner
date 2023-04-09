#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;

    state = NOMAP;

    // Init objects
    localRRT = new RRTObject;
    globalRRT = new RRTObject;

    vehicleParam = new VEHICLE_PARAMETERS;
    mapParam = new MAP_PARAMETERS;

    initObject(localRRT, "local");
    initObject(globalRRT, "global");

    vehicleModel = VehicleModel(vehicleParam);
    mapHandler = MapHandler(mapParam, &vehicleModel);


    // Load ROS parameters
    loadParameters();

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    localRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_local_viz", 10);
    globalRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_global_viz", 10);

    timer = nh.createWallTimer(ros::WallDuration(0.1), &RRTPlanner::timerCallback, this);
    
    ros::spin();
}


void RRTPlanner::initObject(RRTObject* obj, const char* ID)
{
    obj->name = new std::string(1, *ID);

    obj->pathFound = false;
    obj->pathClosed = false;
    
    obj->bestPath = new PATH_TYPE;
    obj->param = new RRT_PARAMETERS;

    // Init objects
    obj->tree = new SearchTree(&vehicleModel, {0.0, 0.0, 0.0}, obj->param);

}

void RRTPlanner::stateMachine()
{
    bool loopClosed;// TODO
    geometry_msgs::Pose2D p; 
    switch(state)
    {
        case NOMAP:
            if (mapHandler.hasMap()) state = LOCALPLANNING;
            break;
        case LOCALPLANNING:
            planLocalRRT();
            p = vehicleModel.getCurrentPose();
            if (vehicleModel.getDistEuclidean({0.0, 0.0, 0.0},{p.x, p.y, 0.0}) < 2)
            {
                state = WAITFORGLOBAL;
                globalRRT->tree->init({0.0, 0.0, 0.0});
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


SearchTreeNode* RRTPlanner::extend(RRTObject* rrt)
{
    SearchTreeNode* newNode;
    bool offCourse;
    SS_VECTOR randState;
    SS_VECTOR newState;
    PATH_TYPE* trajectory;

    // Get random state
    randState = mapHandler.getRandomState(rrt->bestPath, rrt->param);

    // Get nearest node
    SearchTreeNode* nearest = rrt->tree->getNearest(randState);

    // Simulate movement towards new state
    trajectory = vehicleModel.simulateToTarget(nearest->getState(), randState, rrt->param);

    // Check for offCourse
    offCourse = mapHandler.isOffCourse(trajectory, rrt->param);

    // Add node
    if (!offCourse && (trajectory->size() > 0))
    {
        newState = trajectory->back();
        newNode = rrt->tree->addChild(nearest, newState, vehicleModel.getDistanceCost(trajectory));
    }
    else
    {
        newNode = NULL;
    }

    delete trajectory;

    return newNode;
}

bool RRTPlanner::rewire(RRTObject* rrt, SearchTreeNode* newNode)
{
    PATH_TYPE* trajectory;
    std::vector<SearchTreeNode*>* nearbyNodes = rrt->tree->getNearby(newNode, rrt->param->rewireRange);
    std::vector<SearchTreeNode*>::iterator it;
    float newNodeCost = rrt->tree->getAbsCost(newNode);

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = vehicleModel.simulateToTarget(newNode->getState(), (*it)->getState(), rrt->param);
        if (trajectory->size() > 0)
        {
            // Check if new path leads close to new state
            if (vehicleModel.getDistEuclidean(trajectory->back(), (*it)->getState()) < 0.01)
            {
                float segmentCost = vehicleModel.getDistanceCost(trajectory);
                //Compare costs
                if ((newNodeCost + segmentCost) < rrt->tree->getAbsCost(*it))
                {
                    //ROS_INFO_STREAM("" << newNodeCost << "     " << segmentCost << "     " << sTree.getAbsCost(*it));
                    //Rewire if it reduces cost
                    rrt->tree->rewire(*it,newNode);
                    (*it)->changeSegmentCost(segmentCost);
                    //ROS_INFO_STREAM("best dist after:" << sTree.getAbsCost(sTree.getNearest(mapHandler.getGoalState())));
                }
            }
        }
        delete trajectory;
    }
    delete nearbyNodes;

    return false;
}

void RRTPlanner::planLocalRRT(void)
{
    geometry_msgs::Pose2D pose = vehicleModel.getCurrentPose();
    localRRT->tree->init({pose.x, pose.y, pose.theta});
    localRRT->pathFound = false;
    int iteration = 0;

    // TODO
    while((!localRRT->tree->maxNumOfNodesReached()) && (iteration <= localRRT->param->iterations))
    {
        SearchTreeNode * node = extend(localRRT);
        if (node != NULL)
        {   
            rewire(localRRT, node);
            float goalDist = vehicleModel.getDistEuclidean(node->getState(), mapHandler.getGoalState());
            if(goalDist < localRRT->param->goalRadius) localRRT->pathFound = true;
        }
        iteration++;
    }

    if (localRRT->pathFound)
    {
        delete localRRT->bestPath;
        localRRT->bestPath = localRRT->tree->traceBackToRoot(mapHandler.getGoalState());
    }
}

void RRTPlanner::planGlobalRRT(void)
{
    int iteration = 0;

    while((!globalRRT->tree->maxNumOfNodesReached()) && (iteration <= globalRRT->param->iterations))
    {
        SearchTreeNode * node = extend(globalRRT);
        if (node != NULL)
        {   
            bool changed = rewire(globalRRT, node);
            globalRRT->pathFound = changed;
        }
        iteration++;
    }

    if (globalRRT->pathFound)
    {
        delete globalRRT->bestPath;
        globalRRT->bestPath = globalRRT->tree->traceBackToRoot(mapHandler.getGoalState());
    }
}

void RRTPlanner::timerCallback(const ros::WallTimerEvent &event)
{
    stateMachine();
    visualize(localRRT);
    visualize(globalRRT);
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::visualize(RRTObject* rrt)
{
    rrt->markerArray.markers.clear();
    rrt->tree->drawTree(&rrt->markerArray);
    visualizeBestPath(rrt);
    mapHandler.visualizePoints(&rrt->markerArray);
    rrt->markerPublisher.publish(rrt->markerArray);
}

void RRTPlanner::visualizeBestPath(RRTObject* rrt)
{
    // Return if path has been not found yet
    if (!rrt->pathFound) return;


    visualization_msgs::Marker bestPathLine;
        bestPathLine.header.frame_id = "map";
        bestPathLine.header.stamp = ros::Time::now();
        bestPathLine.ns = "rrt_best_path";
        bestPathLine.action = visualization_msgs::Marker::ADD;
        bestPathLine.pose.orientation.w = 1.0;
        bestPathLine.id = 2;
        bestPathLine.type = visualization_msgs::Marker::LINE_STRIP;
        bestPathLine.scale.x = 0.3f;
        bestPathLine.color.r = 0.0f;
        bestPathLine.color.g = 0.5f;
        bestPathLine.color.b = 1.0f;
        bestPathLine.color.a = 1.0f;

    PATH_TYPE::iterator it;
    geometry_msgs::Point coord;

    for (it = rrt->bestPath->begin(); it != rrt->bestPath->end(); it++)
    {
        
        coord.x = (*it)[0];
        coord.y = (*it)[1];
        bestPathLine.points.push_back(coord); 
    }

    rrt->markerArray.markers.emplace_back(bestPathLine);
}

int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);
    return 0;
}


void RRTPlanner::loadParameter(const std::string& topic, float* parameter, const float defaultValue)
{
    if (!ros::param::get(topic, *parameter))
    {
        *parameter = defaultValue;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << topic << " not found."); 
    }
    ROS_INFO_STREAM(topic << "  =  " <<  std::to_string(*parameter)); 
}

void RRTPlanner::loadParameter(const std::string& topic, int* parameter, const int defaultValue)
{
    if (!ros::param::get(topic, *parameter))
    {
        *parameter = defaultValue;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << topic << " not found."); 
    }
    ROS_INFO_STREAM(topic << "  =  " <<  std::to_string(*parameter)); 
}

void RRTPlanner::loadParameter(const std::string& topic, std::string* parameter, const std::string defaultValue)
{
    if (!ros::param::get(topic, *parameter))
    {
        *parameter = defaultValue;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << topic << " not found."); 
    }
    ROS_INFO_STREAM(topic << "  =  " <<  *parameter);     
}

void RRTPlanner::loadParameters(void)
{
    ROS_INFO_STREAM("LOADING PARAMETERS");
    loadParameter("/LOCAL/collisionRange", &localRRT->param->collisionRange, 6.0f);
    loadParameter("/GLOBAL/collisionRange", &globalRRT->param->collisionRange, 6.0f);

    loadParameter("/LOCAL/goalBias", &localRRT->param->goalBias, 0.2f);
    loadParameter("/GLOBAL/goalBias", &globalRRT->param->goalBias, 0.2f);

    loadParameter("/LOCAL/goalRadius", &localRRT->param->goalRadius, 1.0f);
    loadParameter("/GLOBAL/goalRadius", &globalRRT->param->goalRadius, 1.0f);

    loadParameter("/LOCAL/iterations", &localRRT->param->iterations, 500);
    loadParameter("/GLOBAL/iterations", &globalRRT->param->iterations, 500);

    loadParameter("/LOCAL/maxConeDist", &localRRT->param->maxConeDist, 6.0f);
    loadParameter("/GLOBAL/maxConeDist", &globalRRT->param->maxConeDist, 6.0f);

    loadParameter("/LOCAL/maxNumOfNodes", &localRRT->param->maxNumOfNodes, 1000);
    loadParameter("/GLOBAL/maxNumOfNodes", &globalRRT->param->maxNumOfNodes, 1000);

    loadParameter("/LOCAL/maxVelocity", &localRRT->param->maxVelocity, 10.0f);
    loadParameter("/GLOBAL/maxVelocity", &globalRRT->param->maxVelocity, 10.0f);

    loadParameter("/LOCAL/resolution", &localRRT->param->resolution, 0.05f);
    loadParameter("/GLOBAL/resolution", &globalRRT->param->resolution, 0.05f);

    loadParameter("/LOCAL/rewireRange", &localRRT->param->rewireRange, 1.0f);
    loadParameter("/GLOBAL/rewireRange", &globalRRT->param->rewireRange, 1.0f);

    loadParameter("/LOCAL/simulationTimeStep", &localRRT->param->sampleRange, 3);
    loadParameter("/GLOBAL/simulationTimeStep", &globalRRT->param->sampleRange, 3);

    loadParameter("/LOCAL/simulationTimeStep", &localRRT->param->simulationTimeStep, 0.1f);
    loadParameter("/GLOBAL/simulationTimeStep", &globalRRT->param->simulationTimeStep, 0.1f);

    loadParameter("/VEHICLE/track", &vehicleParam->track, 1.2f);
    loadParameter("/VEHICLE/wheelBase", &vehicleParam->wheelBase, 1.54f); 

    loadParameter("/MAP/goalHorizon", &mapParam->goalHorizon, 15.0f);

    // Choose simulation type
    std::string simType;
    loadParameter("/VEHICLE/simType", &simType, "HOLONOMIC");
    if (simType == "HOLONOMIC") vehicleParam->simType = HOLONOMIC;
    else if (simType == "HOLONOMIC_CONSTRAINED") vehicleParam->simType = HOLONOMIC_CONSTRAINED;
    else if (simType == "BICYCLE_SIMPLE") vehicleParam->simType = BICYCLE_SIMPLE;
    else if (simType == "BICYCLE") vehicleParam->simType = BICYCLE;

    // Choose distance calculation type
    std::string distType;
    loadParameter("/VEHICLE/distType", &distType, "EUCLIDEAN");
    if (distType == "EUCLIDEAN") vehicleParam->distType = EUCLIDEAN;
    else if (distType == "SIMULATED") vehicleParam->distType = SIMULATED;

    // Choose distance calculation type
    std::string costType;
    loadParameter("/VEHICLE/costType", &distType, "DISTANCE");
    if (costType == "DISTANCE") vehicleParam->costType = DISTANCE;
    else if (costType == "TIME") vehicleParam->costType = TIME;
}