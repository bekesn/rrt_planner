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

    // Parameter struct
    VEHICLE_PARAMETERS* vehicleParam;

public:
    VehicleModel();
    VehicleModel(VEHICLE_PARAMETERS* par);
    //~VehicleModel();

    // Update vehicle pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Get vehicle pose
    geometry_msgs::Pose2D getCurrentPose();

    // Get parameters
    VEHICLE_PARAMETERS* getParameters();

    // Simulate advancing towards target
    PATH_TYPE* simulateToTarget(SS_VECTOR start, SS_VECTOR goal, RRT_PARAMETERS* param);

    // Distance function
    double distance(SS_VECTOR start, SS_VECTOR goal);

    // Cost function
    double cost(PATH_TYPE* trajectory);

    // SIMULATION FUNCTIONS
    // Holonomic model
    PATH_TYPE* simulateHolonomic(SS_VECTOR start, SS_VECTOR goal, RRT_PARAMETERS* param);

    // Holonomic model with constraints
    PATH_TYPE* simulateHolonomicConstrained(SS_VECTOR start, SS_VECTOR goal, RRT_PARAMETERS* param, float maxAngle = 0.2f);

    // DISTANCE FUNCTIONS
    // Euclidean distance
    double getDistEuclidean(SS_VECTOR start, SS_VECTOR goal);

    // COST FUNCTIONS
    // Cost according to length of trajectory
    double getDistanceCost(PATH_TYPE* trajectory);

    // Cost according to elapsed time
    double getTimeCost(PATH_TYPE* trajectory);

    // Calculate angular difference in rad
    // Anticlockwise
    double angularDifference(SS_VECTOR vehicleState, SS_VECTOR target);
};



#endif //VEHICLEMODEL_H