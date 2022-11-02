#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    distanceFunction = &VehicleModel::getDistEuclidean;
    simulation = &VehicleModel::simulateHolonomic;
}

VehicleModel::VehicleModel(double (VehicleModel::*distFun)(std::vector<double> start, std::vector<double> goal),
                            std::vector<std::vector<double>>* (VehicleModel::*simFun)(std::vector<double> startState, std::vector<double> goalState))
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
    this->currentPose = *msg;
}

geometry_msgs::PoseStamped VehicleModel::getCurrentPose()
{
    return this->currentPose;
}

std::vector<std::vector<double>>* VehicleModel::simulateToTarget(std::vector<double> startState, std::vector<double> goalState)
{
    return (*this.*simulation)(startState, goalState);
}

double VehicleModel::distance(std::vector<double> start, std::vector<double> goal)
{
    return (*this.*distanceFunction)(start, goal);
}

double VehicleModel::getMaximalDistance()
{
    return maxSpeed*simulationTimeStep;
}


std::vector<std::vector<double>>* VehicleModel::simulateHolonomic(std::vector<double> start, std::vector<double> goal)
{
    double distance, ratio, x, y, pathParam;
    int i, numOfStates;
    Eigen::Vector3d startVec(start.data());
    Eigen::Vector3d goalVec(goal.data());
    std::vector<std::vector<double>>* path = new std::vector<std::vector<double>>;

    distance = getDistEuclidean(start, goal);
    ratio = maxSpeed * simulationTimeStep / distance;
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
    double tmp1 = start[0]- goal[0];
    double tmp2 = start[1] - goal[1];
    return(sqrt(tmp1*tmp1 + tmp2*tmp2));
}

double VehicleModel::getDistanceCost(std::vector<std::vector<double>>* trajectory)
{
    std::vector<double> prevState = (*trajectory)[0];
    std::vector<double> currState;
    int size = trajectory->size();
    double length = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*trajectory)[i];
        double dx = currState[0] - prevState[0];
        double dy= currState[1] - prevState[1];
        length += sqrt(dx * dx + dy * dy);
        prevState = currState;
    }
    return length;
}