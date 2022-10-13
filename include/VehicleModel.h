#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

class VehicleModel
{
    geometry_msgs::PoseStamped currentPose;
    double (VehicleModel::*distanceFunction)(std::vector<double> start, std::vector<double> goal);


public:
    VehicleModel();
    ~VehicleModel();

    // Update vehicle pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Get vehicle pose
    geometry_msgs::PoseStamped getCurrentPose();

    // Euclidean distance
    double getDistEuclidean(std::vector<double> start, std::vector<double> goal);

};



#endif //VEHICLEMODEL_H