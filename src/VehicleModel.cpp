#include "VehicleModel.h"

VehicleModel::VehicleModel()
{

}


VehicleModel::~VehicleModel()
{
    
}

void VehicleModel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->currentPose = *msg;
    ROS_INFO_STREAM("pose_callback");
}