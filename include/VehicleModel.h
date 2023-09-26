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
    shared_ptr<SS_VECTOR> currentPose;
    shared_ptr<PATH_TYPE> actualPath;

    // Parameter struct
    unique_ptr<VEHICLE_PARAMETERS> vehicleParam;

public:
    VehicleModel();
    VehicleModel(unique_ptr<VEHICLE_PARAMETERS> par);
    //~VehicleModel();

    // Update vehicle pose and velocity
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Get vehicle pose
    shared_ptr<SS_VECTOR> getCurrentPose(void);

    // Get actual path
    shared_ptr<PATH_TYPE> getActualPath(void) const;

    // Get parameters
    unique_ptr<VEHICLE_PARAMETERS>& getParameters();

    // Simulate advancing towards target
    shared_ptr<PATH_TYPE> simulateToTarget(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
                                           const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier = 1.0f) const;

    // SIMULATION FUNCTIONS
    // Holonomic model
    shared_ptr<PATH_TYPE> simulateHolonomic(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
                                            const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier) const;

    // Holonomic model with constraints
    shared_ptr<PATH_TYPE> simulateHolonomicConstrained(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
                                                       const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier, float maxAngle = 0.5f) const;

    // Simple kinematic bicycle model
    shared_ptr<PATH_TYPE> simulateBicycleSimple(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
                                                const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier) const;

    // DIFFERENTIAL EQUATION SOLVER
    // Runge-Kutta 4th order
    shared_ptr<SS_VECTOR> RK4(const shared_ptr<SS_VECTOR>& startState, const shared_ptr<Control>& controlInput, float dt) const;

    // Visualize actualPath
    void visualize(visualization_msgs::MarkerArray* markerArray) const;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(CEREAL_NVP(vehicleParam)), CEREAL_NVP(currentPose), CEREAL_NVP(actualPath));}
};



#endif //VEHICLEMODEL_H