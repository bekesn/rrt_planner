#include "MapHandler.h"
#include "RRTPlanner.h"
#include "VehicleModel.h"
#include "std_msgs/String.h"
#include <string>



void RRTPlanner::extend(const frt_custom_msgs::SlamStatus::ConstPtr& msg)
{
    //ROS_INFO_STREAM("msg");
    ROS_ERROR("aaaa");
    std::cout<<"aaaa\n";

}

RRTPlanner::RRTPlanner(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;


    ROS_INFO_STREAM("[RRT_PLANNER] Node started.");

    this->sub = nh.subscribe("slam_status", 1, &RRTPlanner::extend, this);

    /*pub = nh.advertise<std_msgs::String>("chatter", 1000);

    while(1)
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world ";
        msg.data = ss.str();
        pub.publish(msg);
    }*/


    ros::spin();
};

RRTPlanner::~RRTPlanner()
{
 
};


int main(int argc, char** argv)
{
    RRTPlanner planner(argc, argv);
    return 0;
}
