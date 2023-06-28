#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <math.h>
#include "Types.h"
#include "StateSpaceSimulated.h"

class VehicleModel
{
    SS_VECTOR currentPose;

    // Parameter struct
    VEHICLE_PARAMETERS* vehicleParam;

public:
    VehicleModel();
    VehicleModel(VEHICLE_PARAMETERS* par);
    //~VehicleModel();

    // Update vehicle pose and velocity
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Get vehicle pose
    SS_VECTOR* getCurrentPose();

    // Get parameters
    VEHICLE_PARAMETERS* getParameters();

    // Simulate advancing towards target
    PATH_TYPE* simulateToTarget(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param);

    // Cost function
    double cost(PATH_TYPE* trajectory, RRT_PARAMETERS* param);

    // SIMULATION FUNCTIONS
    // Holonomic model
    PATH_TYPE* simulateHolonomic(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param);

    // Holonomic model with constraints
    PATH_TYPE* simulateHolonomicConstrained(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param, float maxAngle = 1.0f);

    // Simple kinematic bicycle model
    PATH_TYPE* simulateBicycleSimple(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param);

    // COST FUNCTIONS
    // Cost according to length of trajectory
    double getDistanceCost(PATH_TYPE* trajectory, RRT_PARAMETERS* param);

    // Cost according to elapsed time
    double getTimeCost(PATH_TYPE* trajectory);

    // DIFFERENTIAL EQUATION SOLVER
    // Runge-Kutta 4th order
    SS_VECTOR RK4(SS_VECTOR* startState, Control* controlInput, float dt);
};



#endif //VEHICLEMODEL_H