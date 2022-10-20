#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    distanceFunction = &VehicleModel::getDistEuclidean;
}

VehicleModel::VehicleModel(double (VehicleModel::*distFun)(std::vector<double> start, std::vector<double> goal))
{
    distanceFunction = distFun;
}


void VehicleModel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->currentPose = *msg;
}

geometry_msgs::PoseStamped VehicleModel::getCurrentPose()
{
    return this->currentPose;
}

Eigen::Vector3d VehicleModel::simulateToTarget(std::vector<double> startState, std::vector<double> goalState)
{
    Eigen::Vector3d endState;


    return endState;
}

double VehicleModel::getDistEuclidean(std::vector<double> start, std::vector<double> goal)
{
    double tmp1 = start[0]- goal[0];
    double tmp2 = start[1] - goal[1];
    return(sqrt(tmp1*tmp1 + tmp2*tmp2));
}

double VehicleModel::distance(std::vector<double> start, std::vector<double> goal)
{
    return (*this.*distanceFunction)(start, goal);
}
