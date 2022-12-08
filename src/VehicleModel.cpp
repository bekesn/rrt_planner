#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    distanceFunction = &VehicleModel::getDistEuclidean;
    simulation = &VehicleModel::simulateHolonomic;
}

VehicleModel::VehicleModel(double (VehicleModel::*distFun)(std::vector<double> start, std::vector<double> goal),
                            std::vector<std::vector<double>>* (VehicleModel::*simFun)(std::vector<double> startState, std::vector<double> goalState, double maxConnDist))
{
    distanceFunction = distFun;
    simulation = simFun;

    //TODO
    simulationTimeStep = 0.1;
    maxSpeed = 10;
    resolution = 0.05;
    track = 1.22;

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

std::vector<std::vector<double>>* VehicleModel::simulateToTarget(std::vector<double> startState, std::vector<double> goalState, double maxConnDist)
{
    return (*this.*simulation)(startState, goalState, maxConnDist);
}

double VehicleModel::distance(std::vector<double> start, std::vector<double> goal)
{
    return (*this.*distanceFunction)(start, goal);
}

double VehicleModel::getMaximalDistance()
{
    return maxSpeed*simulationTimeStep;
}


std::vector<std::vector<double>>* VehicleModel::simulateHolonomic(std::vector<double> start, std::vector<double> goal, double maxConndist)
{
    double distance, ratio, x, y, pathParam;
    int i, numOfStates;
    Eigen::Vector3d startVec(start.data());
    Eigen::Vector3d goalVec(goal.data());
    std::vector<std::vector<double>>* path = new std::vector<std::vector<double>>;

    distance = getDistEuclidean(start, goal);
    ratio = maxConndist / distance;
    if (ratio > 1) ratio = 1;
    numOfStates = (int) (ratio*distance/resolution);

    for(i = 0; i < numOfStates; i++)
    {
        x = start[0] + (goal[0] - start[0]) * ratio * (i+1) / (double) numOfStates;
        y = start[1] + (goal[1] - start[1]) * ratio * (i+1) / (double) numOfStates;
        path->push_back({x, y, goal[2]});
    }
    return path;
}

double VehicleModel::getDistEuclidean(std::vector<double> start, std::vector<double> goal)
{
    double dx = start[0]- goal[0];
    double dy = start[1] - goal[1];
    return(sqrt(dx*dx + dy*dy));
}

double VehicleModel::getDistanceCost(std::vector<std::vector<double>>* trajectory)
{
    if(trajectory->size() < 2) return 0;
    
    std::vector<double> prevState = (*trajectory)[0];
    std::vector<double> currState;
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