#include "VehicleModel.h"

VehicleModel::VehicleModel()
{
    actualPath = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    currentPose = shared_ptr<SS_VECTOR> (new SS_VECTOR());
}

VehicleModel::VehicleModel(unique_ptr<VEHICLE_PARAMETERS> par)
{
    vehicleParam = move(par);
    actualPath = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    currentPose = shared_ptr<SS_VECTOR> (new SS_VECTOR());
}


void VehicleModel::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    currentPose = shared_ptr<SS_VECTOR> (new SS_VECTOR(msg->pose.position.x, msg->pose.position.y, yaw, currentPose->v(), currentPose->delta()));

    actualPath->push_back(currentPose);
}


void VehicleModel::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    float v = msg->twist.linear.x;
    v = v < 1 ? 1 : v;      // Avoid slow speeds when planning

    float yawRate = msg->twist.angular.z;
    float delta = atan(vehicleParam->wheelBase * yawRate / v);

    currentPose = shared_ptr<SS_VECTOR> (new SS_VECTOR(currentPose->x(), currentPose->y(), currentPose->theta(), v, delta));
}

shared_ptr<SS_VECTOR> VehicleModel::getCurrentPose(void)
{
    return currentPose;
}

shared_ptr<PATH_TYPE> VehicleModel::getActualPath(void) const
{
    return actualPath;
}

unique_ptr<VEHICLE_PARAMETERS>& VehicleModel::getParameters()
{
    return vehicleParam;
}

shared_ptr<PATH_TYPE> VehicleModel::simulateToTarget(const shared_ptr<SS_VECTOR>& startState, const shared_ptr<SS_VECTOR>& goalState,
    const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier) const
{
    shared_ptr<PATH_TYPE> trajectory;

    switch(vehicleParam->simType)
    {
        case HOLONOMIC:
            trajectory = simulateHolonomic(startState, goalState, param, multiplier);
            break;
        case HOLONOMIC_CONSTRAINED:
            trajectory = simulateHolonomicConstrained(startState, goalState, param, multiplier);
            break;
        case BICYCLE_SIMPLE:
            trajectory = simulateBicycleSimple(startState, goalState, param, multiplier);
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

shared_ptr<PATH_TYPE> VehicleModel::simulateHolonomic(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
    const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier) const
{
    float distance, ratio, x, y;
    int i, numOfStates;
    shared_ptr<PATH_TYPE> path = shared_ptr<PATH_TYPE> (new PATH_TYPE);

    float maxConndist = param->maxVelocity * param->simulationTimeStep;
    distance = start->getDistEuclidean(*goal);
    ratio = multiplier * maxConndist / distance;
    if (ratio > 1) ratio = 1;
    numOfStates = (int) (ratio*distance/param->resolution) + 1;

    for(i = 0; i < numOfStates + 1; i++)
    {
        x = start->x() + (goal->x() - start->x()) * ratio * (i+1) / (float) numOfStates;
        y = start->y() + (goal->y() - start->y()) * ratio * (i+1) / (float) numOfStates;
        path->push_back(make_shared<SS_VECTOR> (SS_VECTOR(x, y, goal->theta())));
    }
    return path;
}

shared_ptr<PATH_TYPE> VehicleModel::simulateHolonomicConstrained(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
    const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier, float maxAngle) const
{

    float distance, angleDiff, ratio, x, y, orientation, dx, dy;
    int i, numOfStates;
    shared_ptr<PATH_TYPE> path = shared_ptr<PATH_TYPE> (new PATH_TYPE);

    float maxConndist = param->maxVelocity * param->simulationTimeStep;
    distance = start->getDistEuclidean(*goal);
    angleDiff = start->getAngleToTarget(*goal);
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

    ratio = multiplier * maxConndist / distance;
    if (ratio > 1) ratio = 1;
    numOfStates = (int) (ratio*distance/param->resolution) + 1;

    for(i = 0; i < numOfStates+1; i++)
    {
        x = start->x() + dx * ratio * i / (float) numOfStates;
        y = start->y() + dy * ratio * i / (float) numOfStates;
        path->push_back(make_shared<SS_VECTOR> (SS_VECTOR(x, y, orientation)));
    }
    return path;
}

shared_ptr<PATH_TYPE> VehicleModel::simulateBicycleSimple(const shared_ptr<SS_VECTOR>& start, const shared_ptr<SS_VECTOR>& goal,
    const unique_ptr<RRT_PARAMETERS>& param, const float& multiplier) const
{
    shared_ptr<PATH_TYPE> path = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    shared_ptr<SS_VECTOR> state = start;
    state->limitVariables(param, vehicleParam);

    bool gettingCloser = true;

    int numOfStates = (int) (multiplier * param->simulationTimeStep * state->v() / param->resolution) + 1;
    float dt = param->simulationTimeStep / (float) numOfStates;

    for (int i = 0; i < numOfStates + 1; i++)
    {
        // If trajectory is not getting closer to goal, break from cycle
        if (state->isGettingCloser(goal, param, vehicleParam))
        {
            path->push_back(state);

            // In the last step, do not simulate
            if (i < numOfStates)
            {
                // Simulate movement
                shared_ptr<Control> controlInput = Control::angleControl(*state, *goal);
                state = RK4(state, controlInput, dt);
                state->limitVariables(param, vehicleParam);
            }
        }
        else
        {
            gettingCloser = false;
            break;
        }
    }

    // If the cycle ended early because of !gettingCloser, the closest point should be searched
    // If path is empty, start state is already !gettingCloser
    if (!gettingCloser && (path->size() > 0))
    {
        // The trajectory endpoint is now the last state which is getting closer
        shared_ptr<SS_VECTOR> prevState = path->back();
        bool closerFound = false;

        for (int i = 0; i < param->simIterations; i++)
        {
            dt = dt / 2.0f;
            
            // Simulate movement
            shared_ptr<Control> controlInput = Control::angleControl(*state, *goal);
            state = RK4(prevState, controlInput, dt);
            state->limitVariables(param, vehicleParam);

            // If new state is still getting closer, 
            bool positiveDerivative = state->isGettingCloser(goal, param, vehicleParam);
            if (positiveDerivative)
            {
                prevState = state;
                closerFound = true;
            }
        }
        
        if (closerFound) path->push_back(state);
    }

    return path;
}

shared_ptr<SS_VECTOR> VehicleModel::RK4(const shared_ptr<SS_VECTOR>& startState, const shared_ptr<Control>& controlInput, float dt) const
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

    return make_shared<SS_VECTOR>((*startState) + k);
}
    
void VehicleModel::visualize(visualization_msgs::MarkerArray* markerArray) const
{
    geometry_msgs::Point coord;

    visualization_msgs::Marker actualPathLine;
        actualPathLine.header.frame_id = "map";
        actualPathLine.header.stamp = ros::Time::now();
        actualPathLine.ns = "vehicle_actual_path";
        actualPathLine.action = visualization_msgs::Marker::ADD;
        actualPathLine.pose.orientation.w = 1.0;
        actualPathLine.id = 2;
        actualPathLine.type = visualization_msgs::Marker::LINE_STRIP;
        actualPathLine.scale.x = 0.05f;
        actualPathLine.color.r = 0.5f;
        actualPathLine.color.g = 0.0f;
        actualPathLine.color.b = 0.5f;
        actualPathLine.color.a = 1.0f;

    PATH_TYPE::iterator pathIterator;
    for (pathIterator = this->actualPath->begin(); pathIterator != this->actualPath->end(); pathIterator++)
    {
        
        coord.x = (*pathIterator)->x();
        coord.y = (*pathIterator)->y();
        actualPathLine.points.push_back(coord);

    }
    
    markerArray->markers.emplace_back(actualPathLine);
}