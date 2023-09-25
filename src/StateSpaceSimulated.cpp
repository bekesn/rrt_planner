#include "StateSpaceSimulated.h"


StateSpaceSimulated::StateSpaceSimulated()
{
    x_ = 0;
    y_ = 0;
    theta_ = 0;
    v_ = 1;
    delta_ = 0;
}

StateSpaceSimulated::StateSpaceSimulated(float x, float y, float theta, float v, float delta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
    v_ = v;
    delta_ = delta;
}

StateSpaceSimulated::StateSpaceSimulated(const StateSpaceSimulated &original)
{
    x_ = original.x_;
    y_ = original.y_;
    theta_ = original.theta_;
    v_ = original.v_;
    delta_ = original.delta_;
}

shared_ptr<StateSpaceSimulated> StateSpaceSimulated::derivative(const shared_ptr<Control>& controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const 
{
    float x, y, theta, v, delta;

    x = v_ * cos(theta_);
    y = v_ * sin(theta_);
    theta = v_ / param->track * tan(delta_);
    v = controlInput->ax;
    delta = controlInput->ddelta;
    
    return shared_ptr<StateSpaceSimulated>(new StateSpaceSimulated(x, y, theta, v, delta));
}

bool StateSpaceSimulated::isGettingCloser(const shared_ptr<StateSpace2D> goalState, const unique_ptr<RRT_PARAMETERS>& rrtParam, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const
{
    // Calculation is based on derivative of getDistOriented
    // Derivative of state variables is calculated as in derivative(), but Control is not needed
    float dx, dy, dtheta;
    dx = v_ * cos(theta_);
    dy = v_ * sin(theta_);
    dtheta = v_ / vehicleParam->track * tan(delta_);

    float numerator = (x_ - goalState->x()) * dx +
                      (y_ - goalState->y()) * dy ;//+
                      //(theta_ - goalState->theta()) * dtheta * rrtParam->thetaWeight;s
    return numerator < 0;
}

void StateSpaceSimulated::limitVariables(const unique_ptr<RRT_PARAMETERS>& rrtParam, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    // Limit v
    if(v_ > rrtParam->maxVelocity) v_ = rrtParam->maxVelocity;
    else if(v_ < 1) v_ = 1;

    // Limit delta
    if(delta_ > vehicleParam->maxDelta) delta_ = vehicleParam->maxDelta;
    else if(delta_ < -vehicleParam->maxDelta) delta_ = -vehicleParam->maxDelta;

    // Limit theta between -PI and PI
    theta_ = std::remainder(theta_, 2*M_PI);
}

StateSpaceSimulated StateSpaceSimulated::operator+ (const StateSpaceSimulated & otherState) const
{
    return StateSpaceSimulated(x_ + otherState.x_, y_ + otherState.y_, theta_ + otherState.theta_, v_ + otherState.v_, delta_ + otherState.delta_);
}

StateSpaceSimulated StateSpaceSimulated::operator- (const StateSpaceSimulated & otherState) const
{
    return StateSpaceSimulated(x_ - otherState.x_, y_ - otherState.y_, theta_ - otherState.theta_, v_ - otherState.v_, delta_ - otherState.delta_);
}

StateSpaceSimulated StateSpaceSimulated::operator* (const float & multiplier) const
{
    return StateSpaceSimulated(x_ * multiplier, y_ * multiplier, theta_ * multiplier, v_ * multiplier, delta_ * multiplier);
}

float StateSpaceSimulated::v() const
{
    return v_;
}

float StateSpaceSimulated::delta() const
{
    return delta_;
}