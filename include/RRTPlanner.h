#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "ros/ros.h"
#include "MapHandler.h"
#include "VehicleModel.h"
#include "frt_custom_msgs/Landmark.h"

class RRTPlanner
{
    ros::Subscriber mapSubscriber;
    ros::Subscriber poseSubscriber;


    MapHandler mapHandler;
    VehicleModel vehicleModel;

public:
    RRTPlanner(int argc, char** argv);
    ~RRTPlanner();
    
    void extend();

};



#endif //RRTPLANNER_H