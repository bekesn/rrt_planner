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

StateSpace2D::StateSpace2D(const StateSpace2D &original)
{
    x_ = original.x_;
    y_ = original.y_;
    theta_ = original.theta_;
}

float StateSpace2D::getDistToTarget(const StateSpace2D* target, const unique_ptr<RRT_PARAMETERS>& param) const
{
    float dx = target->x_ - x_;
    float dy = target->y_ - y_;
    float dtheta = getAngleToTarget(target);
    return sqrt(dx*dx + dy*dy + dtheta * dtheta * param->thetaWeight);
}

float StateSpace2D::getDistOriented(const StateSpace2D* state1, const StateSpace2D* state2, const unique_ptr<RRT_PARAMETERS>& param)
{
    float dx = state1->x_ - state2->x_;
    float dy = state1->y_ - state2->y_;
    float dtheta = state1->getAngleDiff(state2);
    return sqrt(dx*dx + dy*dy + dtheta * dtheta * param->thetaWeight);
}

float StateSpace2D::getDistOriented(const StateSpace2D* otherState, const unique_ptr<RRT_PARAMETERS>& param) const
{
    return StateSpace2D::getDistOriented(this, otherState, param);
}

float StateSpace2D::getDistEuclidean(const std::vector<float> otherState) const
{
    return getDistEuclidean({x_, y_}, otherState);
}

float StateSpace2D::getDistEuclidean(const StateSpace2D* otherState) const
{
    return getDistEuclidean({x_, y_}, {otherState->x_,otherState->y_});
}

float StateSpace2D::getDistEuclidean(const std::vector<float> state1, const std::vector<float> state2)
{
    float dx = state2[0] - state1[0];
    float dy = state2[1] - state1[1];
    return sqrt(dx*dx + dy*dy);
}

float StateSpace2D::getAngleToTarget(const StateSpace2D* target) const
{
    float dy = target->y_ - y_;
    float dx = target->x_ - x_;
    float angle = atan2(dy, dx);
    float diff = angle - theta_;
    float res = std::remainder(diff, 2*M_PI);
    //ROS_INFO_STREAM("" << dy << "  " << dx << "  "  << angle << "  "  << diff << "  "  << res);
    return res;
}

float StateSpace2D::getAngleDiff(const StateSpace2D* otherState) const
{
    return std::remainder(otherState->theta_ - theta_, 2*M_PI);
}

StateSpace2D StateSpace2D::operator+ (const StateSpace2D & otherState) const
{
    return StateSpace2D(x_ + otherState.x_, y_ + otherState.y_, theta_ + otherState.theta_);
}

StateSpace2D StateSpace2D::operator- (const StateSpace2D & otherState) const
{
    return StateSpace2D(x_ - otherState.x_, y_ - otherState.y_, theta_ - otherState.theta_);
}

StateSpace2D StateSpace2D::operator* (const float & multiplier) const
{
    return StateSpace2D(x_ * multiplier, y_ * multiplier, theta_ * multiplier);
}

float StateSpace2D::x(void) const
{
    return x_;
}

float StateSpace2D::y(void) const
{
    return y_;
}

float StateSpace2D::theta(void) const
{
    return theta_;
}