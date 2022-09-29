#include "RRTPlanner.h"
#include "std_msgs/String.h"
#include <string>



RRTPlanner::RRTPlanner(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;

    // Create MapHandler
    //MapHandler mapHandler;
    //VehicleModel vehicleModel;

    // Subscribe to map
    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");
    this->mapSubscriber = nh.subscribe("/map2", 1, &MapHandler::mapCallback, &mapHandler);
    this->poseSubscriber = nh.subscribe("/pose", 1, &VehicleModel::poseCallback, &vehicleModel);

    ros::spin();
}

RRTPlanner::~RRTPlanner()
{
 
}


void RRTPlanner::extend()
{
    

}

int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);
    return 0;
}
