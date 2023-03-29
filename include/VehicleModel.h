#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <math.h>
#include "Types.h"

#define STATESPACE_DIM  3

class VehicleModel
{
    geometry_msgs::Pose2D currentPose;

    // Function pointers
    double (VehicleModel::*distanceFunction)(SS_VECTOR start, SS_VECTOR goal);
    PATH_TYPE* (VehicleModel::*simulation)(SS_VECTOR startState, SS_VECTOR goalState, double maxConnDist);

    // Parameter struct
    RRT_PARAMETERS* param;

public:
    VehicleModel();
    VehicleModel(double (VehicleModel::*distFun)(SS_VECTOR start, SS_VECTOR goal),
                            PATH_TYPE* (VehicleModel::*simFun)(SS_VECTOR startState, SS_VECTOR goalState, double maxConnDist), RRT_PARAMETERS* par);
    //~VehicleModel();

    // Update vehicle pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Get vehicle pose
    geometry_msgs::Pose2D getCurrentPose();

    // Simulate advancing towards target
    PATH_TYPE* simulateToTarget(SS_VECTOR start, SS_VECTOR goal, double maxConnDist);

    // Distance function
    double distance(SS_VECTOR start, SS_VECTOR goal);

    // Get the maximal distance in a timestep
    double getMaximalDistance();

    // SIMULATION FUNCTIONS
    // Holonomic model
    PATH_TYPE* simulateHolonomic(SS_VECTOR start, SS_VECTOR goal, double maxConnDist);

    // DISTANCE FUNCTIONS
    // Euclidean distance
    double getDistEuclidean(std::vector<double> start, std::vector<double> goal);

    //COST FUNCTIONS
    double getDistanceCost(PATH_TYPE* trajectory);


};



#endif //VEHICLEMODEL_H