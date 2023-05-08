#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    
}

VehicleModel::VehicleModel(VEHICLE_PARAMETERS* par)
{
    vehicleParam = par;
}


void VehicleModel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    currentPose = SS_VECTOR(msg->pose.position.x, msg->pose.position.y, yaw);

}

SS_VECTOR* VehicleModel::getCurrentPose()
{
    return new SS_VECTOR(currentPose);
}


VEHICLE_PARAMETERS* VehicleModel::getParameters()
{
    return vehicleParam;
}

PATH_TYPE* VehicleModel::simulateToTarget(SS_VECTOR* startState, SS_VECTOR* goalState, RRT_PARAMETERS* param)
{
    PATH_TYPE* trajectory;

    switch(vehicleParam->simType)
    {
        case HOLONOMIC:
            trajectory = simulateHolonomic(startState, goalState, param);
            break;
        case HOLONOMIC_CONSTRAINED:
            trajectory = simulateHolonomicConstrained(startState, goalState, param);
            break;
        case BICYCLE_SIMPLE:
            trajectory = simulateBicycleSimple(startState, goalState, param);
            break;
        case BICYCLE:
            //TODO
            break;
        default:
            throw std::invalid_argument("Wrong simulation type");
            break;
    }

    return trajectory;
}

double VehicleModel::cost(PATH_TYPE* trajectory, RRT_PARAMETERS* param)
{
    double cost;
    switch(vehicleParam->costType)
    {
        case EUCLIDEAN:
            cost = getDistanceCost(trajectory, param);
            break;
        case SIMULATED:
            cost = getTimeCost(trajectory);
            break;
        default:
            throw std::invalid_argument("Wrong cost calculation type");
            break;
    }

    return cost;
}

PATH_TYPE* VehicleModel::simulateHolonomic(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param)
{
    float distance, ratio, x, y;
    int i, numOfStates;
    PATH_TYPE* path = new PATH_TYPE;

    float maxConndist = param->maxVelocity * param->simulationTimeStep;
    distance = start->distanceToTarget(goal, param);
    ratio = maxConndist / distance;
    if (ratio > 1) ratio = 1;
    numOfStates = (int) (ratio*distance/param->resolution);

    for(i = 0; i < numOfStates; i++)
    {
        x = start->x() + (goal->x() - start->x()) * ratio * (i+1) / (float) numOfStates;
        y = start->y() + (goal->y() - start->y()) * ratio * (i+1) / (float) numOfStates;
        path->push_back(SS_VECTOR(x, y, goal->theta()));
    }
    return path;
}

PATH_TYPE* VehicleModel::simulateHolonomicConstrained(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param, float maxAngle)
{

    float distance, angleDiff, ratio, x, y, orientation, dx, dy;
    int i, numOfStates;
    PATH_TYPE* path = new PATH_TYPE;

    float maxConndist = param->maxVelocity * param->simulationTimeStep;
    distance = start->distanceToTarget(goal, param);
    angleDiff = start->angleToTarget(goal);
    if ((-maxAngle <= angleDiff ) && (angleDiff < maxAngle))
    {
        orientation = atan2((goal->y() - start->y()), (goal->x() - start->x()));
        
    }
    else if ((-M_PI <= angleDiff ) && (angleDiff < -maxAngle))
    {
        orientation = std::remainder(start->theta() - maxAngle, 2.0*M_PI);
    }
    else
    {
        orientation = std::remainder(start->theta() + maxAngle, 2.0*M_PI);
    }


    dx = cos(orientation) * distance;
    dy = sin(orientation) * distance;

    ratio = maxConndist / distance;
    if (ratio > 1) ratio = 1;
    numOfStates = (int) (ratio*distance/param->resolution);

    for(i = 0; i < numOfStates; i++)
    {
        x = start->x() + dx * ratio * (i+1) / (float) numOfStates;
        y = start->y() + dy * ratio * (i+1) / (float) numOfStates;
        path->push_back(SS_VECTOR(x, y, orientation));
    }
    return path;
}

PATH_TYPE* VehicleModel::simulateBicycleSimple(SS_VECTOR* start, SS_VECTOR* goal, RRT_PARAMETERS* param)
{
    PATH_TYPE* path = new PATH_TYPE;
    SS_VECTOR state = *start;

    // TODO kokany
    float dt = param->resolution/param->maxVelocity;
    float t;

    while (t <= param->simulationTimeStep)
    {
        Control* controlInput = Control::angleControl(goal);
        state = RK4(&state, controlInput, dt);
        path->push_back(state);
    }

    return path;
}

double VehicleModel::getDistanceCost(PATH_TYPE* trajectory, RRT_PARAMETERS* param)
{
    if(trajectory->size() < 2) return 0;
    
    SS_VECTOR prevState = (*trajectory)[0];
    SS_VECTOR currState;
    int size = trajectory->size();
    double length = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*trajectory)[i];
        length += prevState.distanceToTarget(&currState, param);
        prevState = currState;
    }
    return length;
}

double VehicleModel::getTimeCost(PATH_TYPE* trajectory)
{
    double elapsed = 0;
    //TODO
    return elapsed;
}

SS_VECTOR VehicleModel::RK4(SS_VECTOR* startState, Control* controlInput, float dt)
{
    SS_VECTOR k1, k2, k3, k4, k;
    k1 = *(startState->derivative(controlInput, vehicleParam)) * dt;
    k2 = *startState + k1*0.5;
    k2 = *(k2.derivative(controlInput, vehicleParam)) * dt;
    k3 = *startState + k2*0.5;
    k3 = *(k3.derivative(controlInput, vehicleParam)) * dt;
    k4 = *startState + k3;
    k4 = *(k4.derivative(controlInput, vehicleParam)) * dt;
    k = (k1 + k2*2 + k3*2 + k4) * (1.0f/6.0f);

    return *startState + k;
}
    