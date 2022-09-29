#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "ros/ros.h"
#include "frt_custom_msgs/SlamStatus.h"
#include "frt_custom_msgs/Landmark.h"

class RRTPlanner
{
    ros::Subscriber sub;
    ros::Publisher pub;
    
    vector<Landmark*> map;

public:
    RRTPlanner(int argc, char** argv);
    ~RRTPlanner();
    
    void extend(const frt_custom_msgs::SlamStatus::ConstPtr& msg);


};



#endif //RRTPLANNER_H