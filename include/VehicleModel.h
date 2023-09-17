#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <math.h>
#include "Trajectory.h"

class VehicleModel
{
    SS_VECTOR currentPose;
    shared_ptr<PATH_TYPE> actualPath;

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
    SS_VECTOR* getCurrentPose(void);

    // Get actual path
    shared_ptr<PATH_TYPE> getActualPath(void) const;

    // Get parameters
    VEHICLE_PARAMETERS* getParameters();

    // Simulate advancing towards target
    shared_ptr<PATH_TYPE> simulateToTarget(SS_VECTOR* start, SS_VECTOR* goal, unique_ptr<RRT_PARAMETERS>& param);

    // SIMULATION FUNCTIONS
    // Holonomic model
    shared_ptr<PATH_TYPE> simulateHolonomic(SS_VECTOR* start, SS_VECTOR* goal, unique_ptr<RRT_PARAMETERS>& param);

    // Holonomic model with constraints
    shared_ptr<PATH_TYPE> simulateHolonomicConstrained(SS_VECTOR* start, SS_VECTOR* goal, unique_ptr<RRT_PARAMETERS>& param, float maxAngle = 0.5f);

    // Simple kinematic bicycle model
    shared_ptr<PATH_TYPE> simulateBicycleSimple(SS_VECTOR* start, SS_VECTOR* goal, unique_ptr<RRT_PARAMETERS>& param);

    // DIFFERENTIAL EQUATION SOLVER
    // Runge-Kutta 4th order
    SS_VECTOR RK4(SS_VECTOR* startState, Control* controlInput, float dt);

    // Visualize actualPath
    void visualize(visualization_msgs::MarkerArray* markerArray);

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(currentPose, actualPath, vehicleParam);}
};



#endif //VEHICLEMODEL_H