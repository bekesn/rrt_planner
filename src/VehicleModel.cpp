#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    distanceFunction = &VehicleModel::getDistEuclidean;
    simulation = &VehicleModel::simulateHolonomic;
}

VehicleModel::VehicleModel(double (VehicleModel::*distFun)(SS_VECTOR start, SS_VECTOR goal),
                            PATH_TYPE* (VehicleModel::*simFun)(SS_VECTOR startState, SS_VECTOR goalState, double maxConnDist), RRT_PARAMETERS* par)
{
    distanceFunction = distFun;
    simulation = simFun;
    param = par;
}


void VehicleModel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    currentPose.x = msg->pose.position.x;
    currentPose.y = msg->pose.position.y;
    currentPose.theta = yaw;

}

geometry_msgs::Pose2D VehicleModel::getCurrentPose()
{
    return currentPose;
}

PATH_TYPE* VehicleModel::simulateToTarget(SS_VECTOR startState, SS_VECTOR goalState, double maxConnDist)
{
    return (*this.*simulation)(startState, goalState, maxConnDist);
}

double VehicleModel::distance(SS_VECTOR start, SS_VECTOR goal)
{
    return (*this.*distanceFunction)(start, goal);
}

double VehicleModel::getMaximalDistance()
{
    return param->maxVelocity * param->simulationTimeStep;
}


PATH_TYPE* VehicleModel::simulateHolonomic(SS_VECTOR start, SS_VECTOR goal, double maxConndist)
{
    double distance, ratio, x, y, pathParam;
    int i, numOfStates;
    Eigen::Vector3d startVec(start.data());
    Eigen::Vector3d goalVec(goal.data());
    PATH_TYPE* path = new PATH_TYPE;

    distance = getDistEuclidean(start, goal);
    ratio = maxConndist / distance;
    if (ratio > 1) ratio = 1;
    numOfStates = (int) (ratio*distance/param->resolution);

    for(i = 0; i < numOfStates; i++)
    {
        x = start[0] + (goal[0] - start[0]) * ratio * (i+1) / (double) numOfStates;
        y = start[1] + (goal[1] - start[1]) * ratio * (i+1) / (double) numOfStates;
        path->push_back({x, y, goal[2]});
    }
    return path;
}

double VehicleModel::getDistEuclidean(SS_VECTOR start, SS_VECTOR goal)
{
    double dx = start[0]- goal[0];
    double dy = start[1] - goal[1];
    return(sqrt(dx*dx + dy*dy));
}

double VehicleModel::getDistanceCost(PATH_TYPE* trajectory)
{
    if(trajectory->size() < 2) return 0;
    
    SS_VECTOR prevState = (*trajectory)[0];
    SS_VECTOR currState;
    int size = trajectory->size();
    double length = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*trajectory)[i];
        length += getDistEuclidean(currState, prevState);
        prevState = currState;
    }
    return length;
}