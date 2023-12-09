#include "DynamicBicycle.h"
#include "Trajectory.h"
#include "Control.h"


DynamicBicycle::DynamicBicycle()
{
    x_ = 0;
    y_ = 0;
    theta_ = 0;
    vx_ = 1;
    delta_ = 0;
}

DynamicBicycle::DynamicBicycle(float x, float y, float theta, float v, float delta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
    vx_ = v;
    delta_ = delta;
}

DynamicBicycle::DynamicBicycle(const DynamicBicycle &original)
{
    x_ = original.x_;
    y_ = original.y_;
    theta_ = original.theta_;
    vx_ = original.vx_;
    delta_ = original.delta_;
}

void DynamicBicycle::initStateSpace(unique_ptr<CONTROL_PARAMETERS> controlParam)
{
    Control<DynamicBicycle>::setParameters(move(controlParam));
}

shared_ptr<DynamicBicycle> DynamicBicycle::derivative(const shared_ptr<Control<DynamicBicycle>>& controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const 
{
    float x, y, theta, v, delta;

    x = vx_ * cos(theta_);
    y = vx_ * sin(theta_);
    theta = vx_ / param->track * tan(delta_);
    v = controlInput->ax;
    delta = controlInput->ddelta;
    
    return shared_ptr<DynamicBicycle>(new DynamicBicycle(x, y, theta, v, delta));
}

void DynamicBicycle::limitVariables(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    // Limit v
    float vxMax = vxLimit(vehicleParam);
    //ROS_INFO_STREAM("" << vxMax << "   " << v_);
    if(vx_ > vxMax) vx_ = vxMax;
    else if(vx_ < 1) vx_ = 1;

    // Limit delta
    if(delta_ > vehicleParam->maxDelta) delta_ = vehicleParam->maxDelta;
    else if(delta_ < -vehicleParam->maxDelta) delta_ = -vehicleParam->maxDelta;

    // Limit theta between -PI and PI
    theta_ = std::remainder(theta_, 2*M_PI);
}

float DynamicBicycle::vxLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const
{
    return vxLimitDynamic(vehicleParam, delta_);
}

float DynamicBicycle::vxLimitNext(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& ddelta, const float& timeStep) const
{
    return vxLimitDynamic(vehicleParam, delta_ + timeStep * ddelta);
}

float DynamicBicycle::vxLimitDynamic(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& delta) const
{
    float vx;
    float absDelta = abs(delta);
    
    if(absDelta < 0.001)
    {
        vx = vehicleParam->maxVelocity;
    }
    else
    {
        vx = min(sqrt(vehicleParam->wheelBase * vehicleParam->maxLatAccel / absDelta), vehicleParam->maxVelocity);
    }
    return vx;
}

float DynamicBicycle::axLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const
{
    float tmp = delta_ / (vehicleParam->wheelBase * vehicleParam->maxLatAccel);
    return vehicleParam->maxLongAccel * sqrt(1 - pow(vx_, 4) * tmp * tmp);
}

DynamicBicycle DynamicBicycle::operator+ (const DynamicBicycle & otherState) const
{
    return DynamicBicycle(x_ + otherState.x_, y_ + otherState.y_, theta_ + otherState.theta_, vx_ + otherState.vx_, delta_ + otherState.delta_);
}

DynamicBicycle DynamicBicycle::operator- (const DynamicBicycle & otherState) const
{
    return DynamicBicycle(x_ - otherState.x_, y_ - otherState.y_, theta_ - otherState.theta_, vx_ - otherState.vx_, delta_ - otherState.delta_);
}

DynamicBicycle DynamicBicycle::operator* (const float & multiplier) const
{
    return DynamicBicycle(x_ * multiplier, y_ * multiplier, theta_ * multiplier, vx_ * multiplier, delta_ * multiplier);
}

float DynamicBicycle::v() const
{
    return vx_;
}

float DynamicBicycle::delta() const
{
    return delta_;
}


shared_ptr<Trajectory<DynamicBicycle>> DynamicBicycle::simulate(const shared_ptr<DynamicBicycle>& start, const shared_ptr<DynamicBicycle>& goal,
    const unique_ptr<RRT_PARAMETERS>& param,  const unique_ptr<VEHICLE_PARAMETERS>& vParam, const float& multiplier)
{
    shared_ptr<Trajectory<DynamicBicycle>> path = shared_ptr<Trajectory<DynamicBicycle>> (new Trajectory<DynamicBicycle>);
    shared_ptr<DynamicBicycle> state = start;
    state->limitVariables(vParam);

    bool gettingCloser = true;

    int numOfStates = (int) (multiplier * param->simulationTimeStep * state->v() / param->resolution) + 1;
    float dt = param->simulationTimeStep / (float) numOfStates;

    for (int i = 0; i < numOfStates + 1; i++)
    {
        // If trajectory is not getting closer to goal, break from cycle
        if (state->isGettingCloser(goal, param, vParam))
        {
            path->push_back(state);

            // In the last step, do not simulate
            if (i < numOfStates)
            {
                // Simulate movement
                shared_ptr<Control<DynamicBicycle>> controlInput = Control<DynamicBicycle>::control(*state, *goal, vParam, dt);
                state = RK4<DynamicBicycle>(state, controlInput, dt, vParam);
                state->limitVariables(vParam);
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
        shared_ptr<DynamicBicycle> prevState = path->back();
        bool closerFound = false;

        for (int i = 0; i < param->simIterations; i++)
        {
            dt = dt / 2.0f;
            
            // Simulate movement
            shared_ptr<Control<DynamicBicycle>> controlInput = Control<DynamicBicycle>::control(*state, *goal, vParam, dt);
            state = RK4(prevState, controlInput, dt, vParam);
            state->limitVariables(vParam);

            // If new state is still getting closer, 
            bool positiveDerivative = state->isGettingCloser(goal, param, vParam);
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