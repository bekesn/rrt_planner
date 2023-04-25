#include "StateSpace2D.h"

StateSpace2D::StateSpace2D()
{
    x_ = 0;
    y_ = 0;
    theta_ = 0;
}

StateSpace2D::StateSpace2D(float x, float y, float theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

float StateSpace2D::distanceToTarget(StateSpace2D* target, RRT_PARAMETERS* param)
{
    float dx = target->x_ - x_;
    float dy = target->y_ - y_;
    float dtheta = angleToTarget(target);
    return sqrt(dx*dx + dy*dy + dtheta * dtheta * param->thetaWeight);
}

float StateSpace2D::getDistEuclidean(std::vector<float> target)
{
    return getDistEuclidean({x_, y_}, target);
}

double StateSpace2D::getDistEuclidean(const std::vector<float> start, const std::vector<float> goal)
{
    float dx = start[0] - goal[0];
    float dy = start[1] - goal[1];
    return sqrt(dx*dx + dy*dy);
}

double StateSpace2D::angleToTarget(StateSpace2D* target)
{
    auto dy = target->y_ - y_;
    auto dx = target->x_ - x_;
    auto angle = atan2(dy, dx);
    auto diff = angle - theta_;
    auto res = std::remainder(diff, 2*M_PI);
    //ROS_INFO_STREAM("" << dy << "  " << dx << "  "  << angle << "  "  << diff << "  "  << res);
    return res;
}

StateSpace2D StateSpace2D::operator+ (const StateSpace2D & otherState) const
{
    return StateSpace2D(x_ + otherState.x_, y_ + otherState.y_, (theta_ + otherState.theta_)/2);
}

StateSpace2D StateSpace2D::operator- (const StateSpace2D & otherState) const
{
    return StateSpace2D(x_ - otherState.x_, y_ - otherState.y_, (theta_ - otherState.theta_)/2);
}

StateSpace2D StateSpace2D::operator* (const float & multiplier) const
{
    return StateSpace2D(x_ * multiplier, y_ * multiplier, theta_);
}

float StateSpace2D::x()
{
    return x_;
}

float StateSpace2D::y()
{
    return y_;
}

float StateSpace2D::theta()
{
    return theta_;
}