#include "StateSpaceSimulated.h"


StateSpaceSimulated::StateSpaceSimulated()
{
    x_ = 0;
    y_ = 0;
    theta_ = 0;
    v_ = 0;
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

StateSpaceSimulated* StateSpaceSimulated::derivative(Control* controlInput, VEHICLE_PARAMETERS* param)
{
    float x, y, theta, v, delta;

    x = v_ * cos(theta_);
    y = v_ * sin(theta_);
    theta_ = v_ / param->track * tan(delta_);
    v = controlInput->dv;
    delta = controlInput->ddelta;
    
    return new StateSpaceSimulated(x, y, theta, v, delta);
}

StateSpaceSimulated StateSpaceSimulated::operator+ (const StateSpaceSimulated & otherState) const
{
    return StateSpaceSimulated(x_ + otherState.x_, y_ + otherState.y_, theta_ + otherState.theta_, v_ + otherState.theta_, delta_ + otherState.delta_);
}

StateSpaceSimulated StateSpaceSimulated::operator- (const StateSpaceSimulated & otherState) const
{
    return StateSpaceSimulated(x_ - otherState.x_, y_ - otherState.y_, theta_ - otherState.theta_, v_ - otherState.theta_, delta_ - otherState.delta_);
}

StateSpaceSimulated StateSpaceSimulated::operator* (const float & multiplier) const
{
    return StateSpaceSimulated(x_ * multiplier, y_ * multiplier, theta_ * multiplier, v_ * multiplier, delta_ * multiplier);
}