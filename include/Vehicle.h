#ifndef VEHICLE_H
#define VEHICLE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <math.h>
#include "Types.h"
#include "Trajectory.h"

template<class StateSpaceVector>
class Vehicle
{
    DynamicBicycle currentPose;
    shared_ptr<Trajectory<StateSpaceVector>> actualPath;

    // Parameter struct
    unique_ptr<VEHICLE_PARAMETERS> vehicleParam;

public:
    Vehicle();
    Vehicle(unique_ptr<VEHICLE_PARAMETERS> par);

    // Update vehicle pose and velocity
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

    // Get vehicle pose
    shared_ptr<StateSpaceVector> getCurrentPose(void);

    // Get actual path
    shared_ptr<Trajectory<StateSpaceVector>> getActualPath(void) const;

    // Get parameters
    unique_ptr<VEHICLE_PARAMETERS>& getParameters();

    // Visualize actualPath
    void visualize(visualization_msgs::MarkerArray* markerArray) const;
};



#endif //VEHICLE_H