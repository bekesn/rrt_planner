#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    distanceFunction = &VehicleModel::getDistEuclidean;
}


VehicleModel::~VehicleModel()
{
    
}

void VehicleModel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->currentPose = *msg;
}

geometry_msgs::PoseStamped VehicleModel::getCurrentPose()
{
    return this->currentPose;
}

double VehicleModel::getDistEuclidean(std::vector<double> start, std::vector<double> goal)
{
    double tmp1 = start.front() - goal.front();
    double tmp2 = start.back() - goal.back();
    //ROS_INFO_STREAM("dist: " << sqrt(tmp1*tmp1 + tmp2*tmp2));
    return(sqrt(tmp1*tmp1 + tmp2*tmp2));
}
