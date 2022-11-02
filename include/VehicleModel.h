#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <math.h>

#define STATESPACE_DIM  3

class VehicleModel
{
    geometry_msgs::PoseStamped currentPose;

    // Function pointers
    double (VehicleModel::*distanceFunction)(std::vector<double> start, std::vector<double> goal);
    std::vector<std::vector<double>>* (VehicleModel::*simulation)(std::vector<double> startState, std::vector<double> goalState);

    // Parameters
    float simulationTimeStep;
    float maxSpeed;
    float resolution;

public:
    float track;
    
    VehicleModel();
    VehicleModel(double (VehicleModel::*distFun)(std::vector<double> start, std::vector<double> goal),
                            std::vector<std::vector<double>>* (VehicleModel::*simFun)(std::vector<double> startState, std::vector<double> goalState));
    //~VehicleModel();

    // Update vehicle pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // Get vehicle pose
    geometry_msgs::PoseStamped getCurrentPose();

    // Simulate advancing towards target
    std::vector<std::vector<double>>* simulateToTarget(std::vector<double> start, std::vector<double> goal);

    // Distance function
    double distance(std::vector<double> start, std::vector<double> goal);

    // Get the maximal distance in a timestep
    double getMaximalDistance();

    // SIMULATION FUNCTIONS
    // Holonomic model
    std::vector<std::vector<double>>* simulateHolonomic(std::vector<double> start, std::vector<double> goal);

    // DISTANCE FUNCTIONS
    // Euclidean distance
    double getDistEuclidean(std::vector<double> start, std::vector<double> goal);

    //COST FUNCTIONS
    double getDistanceCost(std::vector<std::vector<double>>* trajectory);
};



#endif //VEHICLEMODEL_H