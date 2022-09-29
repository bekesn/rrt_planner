#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class VehicleModel
{
    geometry_msgs::PoseStamped currentPose;


public:
    VehicleModel();
    ~VehicleModel();

    // Update vehicle pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);


};



#endif //VEHICLEMODEL_H