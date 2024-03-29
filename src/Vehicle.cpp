#include "Vehicle.h"

template<class StateSpaceVector>
Vehicle<StateSpaceVector>::Vehicle()
{
    actualPath = shared_ptr<Trajectory<StateSpaceVector>> (new Trajectory<StateSpaceVector>);
    currentPose = DynamicBicycle();
}

template<class StateSpaceVector>
Vehicle<StateSpaceVector>::Vehicle(unique_ptr<VEHICLE_PARAMETERS> par) : Vehicle()
{
    vehicleParam = move(par);
}

template<class StateSpaceVector>
void Vehicle<StateSpaceVector>::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    currentPose = DynamicBicycle(msg->pose.position.x, msg->pose.position.y, yaw, currentPose.vx(), currentPose.delta());

    actualPath->push_back(make_shared<StateSpaceVector>((StateSpaceVector) currentPose));
}

template<class StateSpaceVector>
void Vehicle<StateSpaceVector>::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    float vx = msg->twist.linear.x;
    vx = vx < 1 ? 1 : vx;      // Avoid slow speeds when planning

    float yawRate = msg->twist.angular.z;
    float delta = atan(vehicleParam->wheelBase * yawRate / vx);

    currentPose = DynamicBicycle(currentPose.x(), currentPose.y(), currentPose.theta(), vx, delta);
}

template<class StateSpaceVector>
shared_ptr<StateSpaceVector> Vehicle<StateSpaceVector>::getCurrentPose(void)
{
    return make_shared<StateSpaceVector>((StateSpaceVector) currentPose);
}

template<class StateSpaceVector>
shared_ptr<Trajectory<StateSpaceVector>> Vehicle<StateSpaceVector>::getActualPath(void) const
{
    return actualPath;
}

template<class StateSpaceVector>
unique_ptr<VEHICLE_PARAMETERS>& Vehicle<StateSpaceVector>::getParameters()
{
    return vehicleParam;
}
   
template<class StateSpaceVector> 
void Vehicle<StateSpaceVector>::visualize(visualization_msgs::MarkerArray* markerArray) const
{
    float relativeVelocity;
    std_msgs::ColorRGBA varColor;
    varColor.r = 0;
    varColor.g = 0;
    varColor.b = 0;
    varColor.a = 1;

    visualization_msgs::Marker actualPathLine;
        actualPathLine.header.frame_id = "map";
        actualPathLine.header.stamp = ros::Time::now();
        actualPathLine.ns = "vehicle_actual_path";
        actualPathLine.action = visualization_msgs::Marker::ADD;
        actualPathLine.pose.orientation.w = 1.0;
        actualPathLine.id = 2;
        actualPathLine.type = visualization_msgs::Marker::LINE_STRIP;
        actualPathLine.scale.x = 0.4f;
        actualPathLine.color.r = 0.5f;
        actualPathLine.color.g = 0.0f;
        actualPathLine.color.b = 0.5f;
        actualPathLine.color.a = 1.0f;

    actualPath->visualize(&actualPathLine);   
    typename Trajectory<StateSpaceVector>::iterator itT;
    for (itT = actualPath->begin(); itT != actualPath->end(); itT++)
    {
        relativeVelocity = (*itT)->vx() / vehicleParam->maxVelocity;
        if(relativeVelocity > 1.0f) relativeVelocity = 1.0f;
        //varColor.r = relativeVelocity;
        //varColor.g = 1 - relativeVelocity;
        //actualPathLine.colors.push_back(varColor);
    } 
    markerArray->markers.emplace_back(actualPathLine);
}

// Define classes
template class Vehicle<StateSpace2D>;
template class Vehicle<KinematicBicycle>;
template class Vehicle<DynamicBicycle>;