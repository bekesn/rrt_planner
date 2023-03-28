#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    // Init
    pathFound = false;
    bestPath = new std::vector<std::vector<double>>;

    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;

    // Init objects
    vehicleModel = VehicleModel(&VehicleModel::getDistEuclidean, &VehicleModel::simulateHolonomic);
    mapHandler = MapHandler(&vehicleModel);
    sTree = SearchTree(&vehicleModel, {0.0, 0.0, 0.0});

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_viz", 10);


    timer = nh.createWallTimer(ros::WallDuration(0.1), &RRTPlanner::timerCallback, this);

    goalRadius = 0.5;
    
    ros::spin();
}


SearchTreeNode* RRTPlanner::extend()
{
    SearchTreeNode* newNode;
    bool offCourse;
    std::vector<double> randState;
    std::vector<double> newState;
    std::vector<std::vector<double>>* trajectory;

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
    double maxConnDist = vehicleModel.getMaximalDistance();
    std::vector<std::vector<double>>* trajectory;
    std::vector<SearchTreeNode*>* nearbyNodes = sTree.getNearby(newNode, 3*maxConnDist);
    std::vector<SearchTreeNode*>::iterator it;
    float newNodeCost = sTree.getAbsCost(newNode);

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = vehicleModel.simulateToTarget(newNode->getState(), (*it)->getState(), 3*maxConnDist);
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

void RRTPlanner::planLocalRRT()
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
            if(goalDist < goalRadius)
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

void RRTPlanner::planGlobalRRT()
{
    // TODO
    for(int i = 0; i < 300; i++)
    {
        extend();
    }
}

void RRTPlanner::timerCallback(const ros::WallTimerEvent &event)
{
    if (!mapHandler.hasMap()) return;

    bool loopClosed = false; // TODO
    if(!loopClosed)
    {
        planLocalRRT();
    }
    else
    {
        planGlobalRRT();
    }
    visualize();
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::visualize()
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

    std::vector<std::vector<double>>::iterator it;
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
