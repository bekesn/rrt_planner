#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    // Init
    pathFound = false;
    pathClosed = false;
    state = NOMAP;
    bestPath = new PATH_TYPE;

    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;

    // Load ROS parameters
    loadParameters();

    // Init objects
    vehicleModel = VehicleModel(&VehicleModel::getDistEuclidean, &VehicleModel::simulateHolonomic, param);
    mapHandler = MapHandler(&vehicleModel);
    sTree = SearchTree(&vehicleModel, {0.0, 0.0, 0.0}, param);

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_viz", 10);


    timer = nh.createWallTimer(ros::WallDuration(0.1), &RRTPlanner::timerCallback, this);
    
    ros::spin();
}

void RRTPlanner::stateMachine()
{
    bool loopClosed = false;// TODO
    switch(state)
    {
        case NOMAP:
            if (mapHandler.hasMap()) state = LOCALPLANNING;
            break;
        case LOCALPLANNING:
            planLocalRRT();
            if(loopClosed) state = WAITFORGLOBAL;
        case WAITFORGLOBAL:
            //TODO
            planLocalRRT();
            if(pathClosed) state = GLOBALPLANNING;
        case GLOBALPLANNING:
            planGlobalRRT();
            break;
    }
}


SearchTreeNode* RRTPlanner::extend(void)
{
    SearchTreeNode* newNode;
    bool offCourse;
    SS_VECTOR randState;
    SS_VECTOR newState;
    PATH_TYPE* trajectory;

    // Get random state
    randState = mapHandler.getRandomState(bestPath);

    // Get nearest node
    SearchTreeNode* nearest = sTree.getNearest(randState);

    // Simulate movement towards new state
    trajectory = vehicleModel.simulateToTarget(nearest->getState(), randState, vehicleModel.getMaximalDistance());

    // Check for offCourse
    offCourse = mapHandler.isOffCourse(trajectory);

    // Add node
    if (!offCourse && (trajectory->size() > 0))
    {
        newState = trajectory->back();
        newNode = sTree.addChild(nearest, newState, vehicleModel.getDistanceCost(trajectory));
    }
    else
    {
        newNode = NULL;
    }
    return newNode;
}

bool RRTPlanner::rewire(SearchTreeNode* newNode)
{
    PATH_TYPE* trajectory;
    std::vector<SearchTreeNode*>* nearbyNodes = sTree.getNearby(newNode, param->rewireRange);
    std::vector<SearchTreeNode*>::iterator it;
    float newNodeCost = sTree.getAbsCost(newNode);

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = vehicleModel.simulateToTarget(newNode->getState(), (*it)->getState(), param->rewireRange);
        if (trajectory->size() > 0)
        {
            // Check if new path leads close to new state
            if (vehicleModel.getDistEuclidean(trajectory->back(), (*it)->getState()) < 0.01)
            {
                float segmentCost = vehicleModel.getDistanceCost(trajectory);
                //Compare costs
                if ((newNodeCost + segmentCost) < sTree.getAbsCost(*it))
                {
                    //ROS_INFO_STREAM("" << newNodeCost << "     " << segmentCost << "     " << sTree.getAbsCost(*it));
                    //Rewire if it reduces cost
                    sTree.rewire(*it,newNode);
                    (*it)->changeSegmentCost(segmentCost);
                    //ROS_INFO_STREAM("best dist after:" << sTree.getAbsCost(sTree.getNearest(mapHandler.getGoalState())));
                }
            }
        }
    }
    return false;
}

void RRTPlanner::planLocalRRT(void)
{
    geometry_msgs::Pose2D pose = vehicleModel.getCurrentPose();
    sTree.init({pose.x, pose.y, 0.0});
    pathFound = false;
    int iteration = 0;

    // TODO
    while((!sTree.maxNumOfNodesReached()) && (iteration <= 500))
    {
        SearchTreeNode * node = extend();
        if (node != NULL)
        {   
            rewire(node);
            float goalDist = vehicleModel.getDistEuclidean(node->getState(), mapHandler.getGoalState());
            if(goalDist < param->goalRadius)
            pathFound = true;
        }
        iteration++;
    }

    if (pathFound)
    {
        bestPath->clear();
        bestPath = sTree.traceBackToRoot(mapHandler.getGoalState());
    }

    ROS_INFO_STREAM("dist: " << sTree.getAbsCost(sTree.getNearest(mapHandler.getGoalState())));
}

void RRTPlanner::planGlobalRRT(void)
{
    // TODO
    for(int i = 0; i < 300; i++)
    {
        extend();
    }
}

void RRTPlanner::timerCallback(const ros::WallTimerEvent &event)
{
    stateMachine();
    visualize();
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::visualize(void)
{
    markerArray.markers.clear();
    sTree.drawTree(&markerArray);
    visualizeBestPath(&markerArray);
    mapHandler.visualizePoints(&markerArray);
    markerPublisher.publish(markerArray);
}

void RRTPlanner::visualizeBestPath(visualization_msgs::MarkerArray* mArray)
{
    // Return if path has been not found yet
    if (!pathFound) return;


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

    for (it = bestPath->begin(); it != bestPath->end(); it++)
    {
        
        coord.x = (*it)[0];
        coord.y = (*it)[1];
        bestPathLine.points.push_back(coord); 
    }

    mArray->markers.emplace_back(bestPathLine);
}

int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);
    return 0;
}

void RRTPlanner::loadParameters(void)
{
    if (!ros::param::get("/RRT/collisionRange", param->collisionRange))
    {
        param->collisionRange = 6.0f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/collisionRange not found."); 
    }

    if (!ros::param::get("/RRT/goalBias", param->goalBias))
    {
        param->goalBias = 0.2f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/goalBias not found."); 
    }

    if (!ros::param::get("/RRT/goalHorizon", param->goalHorizon))
    {
        param->goalHorizon = 6.0f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/goalHorizon not found."); 
    }

    if (!ros::param::get("/RRT/goalRadius", param->goalRadius))
    {
        param->goalRadius = 1.0f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/goalRadius not found."); 
    }

    if (!ros::param::get("/RRT/maxConeDist", param->maxConeDist))
    {
        param->maxConeDist = 6.0f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/maxConeDist not found."); 
    }

    if (!ros::param::get("/RRT/maxNumOfNodes", param->maxNumOfNodes))
    {
        param->maxNumOfNodes = 1000;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/maxNumOfNodes not found."); 
    }

    if (!ros::param::get("/VehicleModel/maxVelocity", param->maxVelocity))
    {
        param->maxVelocity = 10.0f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/maxVelocity not found."); 
    }

    if (!ros::param::get("/VehicleModel/resolution", param->resolution))
    {
        param->resolution = 0.05f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/resolution not found."); 
    }
    if (!ros::param::get("/RRT/rewireRange", param->rewireRange))
    {
        param->rewireRange = 9.0f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/rewireRange not found."); 
    }

    if (!ros::param::get("/VehicleModel/simulationTimeStep", param->simulationTimeStep))
    {
        param->simulationTimeStep = 0.1f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/simulationTimeStep not found."); 
    }

    if (!ros::param::get("/VehicleModel/track", param->track))
    {
        param->track = 1.2f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/track not found."); 
    }

    if (!ros::param::get("/VehicleModel/wheelBase", param->wheelBase))
    {
        param->wheelBase = 0.1f;
        ROS_ERROR_STREAM("Parameter " << ros::this_node::getName() << "/wheelBase not found."); 
    }
}