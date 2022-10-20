#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <math.h>

class VehicleModel
{
    geometry_msgs::PoseStamped currentPose;
    double (VehicleModel::*distanceFunction)(std::vector<double> start, std::vector<double> goal);


public:
    VehicleModel();
    VehicleModel(double (VehicleModel::*distFun)(std::vector<double> start, std::vector<double> goal));
    //~VehicleModel();

    // Update vehicle pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Get vehicle pose
    geometry_msgs::PoseStamped getCurrentPose();

    // Simulate advancing towards target
    Eigen::Vector3d simulateToTarget(std::vector<double> start, std::vector<double> goal);

    // Euclidean distance
    double getDistEuclidean(std::vector<double> start, std::vector<double> goal);

    // Distance function
    double distance(std::vector<double> start, std::vector<double> goal);
};



#endif //VEHICLEMODEL_H