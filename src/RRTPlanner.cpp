#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;

    vehicleModel = VehicleModel(&VehicleModel::getDistEuclidean);

    sTree = SearchTree(&vehicleModel, {0.0, 10.0});

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    mapSubscriber = nh.subscribe("/map2", 1, &MapHandler::mapCallback, &mapHandler);
    poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);
    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_viz", 10);


    timer = nh.createWallTimer(ros::WallDuration(1), &RRTPlanner::visualize, this);

    
    ros::spin();
}


void RRTPlanner::extend()
{
    
}

void RRTPlanner::visualize(const ros::WallTimerEvent &event)
{
    double x = rand()%1000 / 100.0 - 5;
    double y = rand()%1000 / 100.0 - 5;
    SearchTreeNode* nearest = sTree.getNearest({x, y});
    sTree.addChild(nearest, {x, y});
    sTree.drawTree(&markerArray);
    markerPublisher.publish(markerArray);
    ROS_INFO_STREAM("-------------");
}

int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);
    return 0;
}
