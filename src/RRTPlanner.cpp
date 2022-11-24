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
    sTree = SearchTree(&vehicleModel, {10.0, 0.0, 0.0});

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_viz", 10);


    timer = nh.createWallTimer(ros::WallDuration(0.1), &RRTPlanner::timerCallback, this);

    
    ros::spin();
}


bool RRTPlanner::extend()
{
    bool offCourse;
    std::vector<double> randState;
    std::vector<double> newState;
    std::vector<std::vector<double>>* trajectory;

    // Get random state
    randState = mapHandler.getRandomState(bestPath);

    // Get nearest node
    SearchTreeNode* nearest = sTree.getNearest(randState);

    // Simulate movement towards new state
    trajectory = vehicleModel.simulateToTarget(nearest->getState(), randState);

    // Check for offCourse
    offCourse = mapHandler.isOffCourse(trajectory);

    // Add node
    if (!offCourse && (trajectory->size() > 0))
    {
        SearchTreeNode* newNode;
        newState = trajectory->back();
        newNode = sTree.addChild(nearest, newState, vehicleModel.getDistanceCost(trajectory));

        if (newNode)
        {
            rewire(newNode);

            if (vehicleModel.getDistEuclidean(newState, mapHandler.getGoalState()) < 0.5)
            {
                return true;
            }
        }
        
    }
    return false;
}

bool RRTPlanner::rewire(SearchTreeNode* newNode)
{
    std::vector<std::vector<double>>* trajectory;
    std::vector<SearchTreeNode*>* nearbyNodes = sTree.getNearby(newNode, vehicleModel.getMaximalDistance());
    std::vector<SearchTreeNode*>::iterator it;
    float newNodeCost = sTree.getAbsCost(newNode);

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = vehicleModel.simulateToTarget(newNode->getState(), (*it)->getState());
        if (trajectory->size() > 0)
        {
            // Check if new path leads close to new state
            if (vehicleModel.getDistEuclidean(trajectory->back(), (*it)->getState()) < 0.01)
            {
                float segmentCost = vehicleModel.getDistanceCost(trajectory);
                //Compare costs
                if ((newNodeCost + segmentCost) < sTree.getAbsCost(*it))
                {
                    ROS_INFO_STREAM("" << newNodeCost << "     " << segmentCost << "     " << sTree.getAbsCost(*it));
                    //Rewire if it reduces cost
                    sTree.rewire(*it,newNode);
                    (*it)->changeSegmentCost(segmentCost);
                    ROS_INFO_STREAM("best dist after:" << sTree.getAbsCost(sTree.getNearest(mapHandler.getGoalState())));
                }
            }
        }
    }
    return false;
}

void RRTPlanner::planOpenTrackRRT()
{
    geometry_msgs::Pose2D pose = vehicleModel.getCurrentPose();
    sTree.reset({pose.x, pose.y, 0.0});
    pathFound = false;
    int iteration = 0;

    if (!mapHandler.hasMap()) return;

    // TODO
    while((!sTree.maxNumOfNodesReached()) && (iteration <= 2500))
    {
        bool closeToGoal = extend();
        if (closeToGoal)
        {
            pathFound = true;
            break;
        }
        iteration++;
    }

    if (pathFound)
    {
        bestPath->clear();
        bestPath = sTree.traceBackToRoot(mapHandler.getGoalState());
    }

    visualize();
    ROS_INFO_STREAM("dist: " << sTree.getAbsCost(sTree.getNearest(mapHandler.getGoalState())));
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::planClosedTrackRRT()
{
    sTree.reset({10.0, 0.0, 0.0});

    // TODO
    for(int i = 0; i < 300; i++)
    {
        extend();
    }
    visualize();
    ROS_INFO_STREAM("-------------");
}

void RRTPlanner::timerCallback(const ros::WallTimerEvent &event)
{
    planOpenTrackRRT();
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
