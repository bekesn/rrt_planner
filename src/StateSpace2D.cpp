#include "StateSpace2D.h"
#include "Trajectory.h"

StateSpace2D::StateSpace2D()
{
    x_ = 0;
    y_ = 0;
    theta_ = 0;
}

StateSpace2D::StateSpace2D(float x, float y, float theta, float vx)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
    vx_ = vx;
}

StateSpace2D::StateSpace2D(const StateSpace2D &original)
{
    x_ = original.x_;
    y_ = original.y_;
    theta_ = original.theta_;
    vx_ = original.vx_;
}

void StateSpace2D::initStateSpace(unique_ptr<CONTROL_PARAMETERS> controlParam)
{

}

float StateSpace2D::getDistToTarget2(const StateSpace2D& target, const unique_ptr<RRT_PARAMETERS>& param) const
{
    float dx = target.x_ - x_;
    float dy = target.y_ - y_;
    float dtheta = getAngleToTarget(target);
    return dx*dx + dy*dy + dtheta * dtheta * param->psiWeight;
}

float StateSpace2D::getDistOriented2(const StateSpace2D& otherState, const unique_ptr<RRT_PARAMETERS>& param) const
{
    float dx = x_ - otherState.x_;
    float dy = y_ - otherState.y_;
    return dx*dx + dy*dy;
}

float StateSpace2D::getDistEuclidean(const std::vector<float> otherState) const
{
    float dx = otherState[0] - x_;
    float dy = otherState[1] - y_;
    return sqrt(dx*dx + dy*dy);
}

float StateSpace2D::getDistEuclidean2(const std::vector<float> otherState) const
{
    float dx = otherState[0] - x_;
    float dy = otherState[1] - y_;
    return dx*dx + dy*dy;
}

float StateSpace2D::getDistEuclidean(const StateSpace2D& otherState) const
{
    float dx = otherState.x_ - x_;
    float dy = otherState.y_ - y_;
    return sqrt(dx*dx + dy*dy);
}

float StateSpace2D::getDistEuclidean2(const StateSpace2D& otherState) const
{
    float dx = otherState.x_ - x_;
    float dy = otherState.y_ - y_;
    return dx*dx + dy*dy;
}

float StateSpace2D::getDistEuclidean(const std::vector<float> state1, const std::vector<float> state2)
{
    float dx = state2[0] - state1[0];
    float dy = state2[1] - state1[1];
    return sqrt(dx*dx + dy*dy);
}

float StateSpace2D::getDistEuclidean2(const std::vector<float> state1, const std::vector<float> state2)
{
    float dx = state2[0] - state1[0];
    float dy = state2[1] - state1[1];
    return dx*dx + dy*dy;
}

float StateSpace2D::getAngleToTarget(const StateSpace2D& target) const
{
    float dy = target.y_ - y_;
    float dx = target.x_ - x_;
    float angle = atan2(dy, dx);
    float diff = angle - theta_;
    float res = std::remainder(diff, 2*M_PI);
    //ROS_INFO_STREAM("" << dy << "  " << dx << "  "  << angle << "  "  << diff << "  "  << res);
    return res;
}

float StateSpace2D::getAngleDiff(const StateSpace2D& otherState) const
{
    return std::remainder(otherState.theta_ - theta_, 2*M_PI);
}

StateSpace2D StateSpace2D::operator+ (const StateSpace2D & otherState) const
{
    return StateSpace2D(x_ + otherState.x_, y_ + otherState.y_, theta_ + otherState.theta_, vx_ + otherState.vx_);
}

StateSpace2D StateSpace2D::operator- (const StateSpace2D & otherState) const
{
    return StateSpace2D(x_ - otherState.x_, y_ - otherState.y_, theta_ - otherState.theta_,  vx_ - otherState.vx_);
}

StateSpace2D StateSpace2D::operator* (const float & multiplier) const
{
    return StateSpace2D(x_ * multiplier, y_ * multiplier, theta_ * multiplier, vx_ * multiplier);
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

float StateSpace2D::vx(void) const
{
    return vx_;
}

shared_ptr<Trajectory<StateSpace2D>> StateSpace2D::simulate(const shared_ptr<StateSpace2D>& start, const shared_ptr<StateSpace2D>& goal,
    const unique_ptr<RRT_PARAMETERS>& param,  const unique_ptr<VEHICLE_PARAMETERS>& vParam, const float& multiplier)
{
    float maxAngle = 0.5;

    float distance, angleDiff, ratio, x, y, orientation, dx, dy;
    int i, numOfStates;
    shared_ptr<Trajectory<StateSpace2D>> path = shared_ptr<Trajectory<StateSpace2D>> (new Trajectory<StateSpace2D>);

    float maxConndist = vParam->maxVelocity * param->simulationTimeStep;
    distance = start->getDistEuclidean(*goal);
    angleDiff = start->getAngleToTarget(*goal);
    if ((-maxAngle <= angleDiff ) && (angleDiff < maxAngle))
    {
        orientation = atan2((goal->y() - start->y()), (goal->x() - start->x()));
        
    }
    else if ((-M_PI <= angleDiff ) && (angleDiff < -maxAngle))
    {
        orientation = remainder(start->theta() - maxAngle, 2.0*M_PI);
    }
    else
    {
        orientation = remainder(start->theta() + maxAngle, 2.0*M_PI);
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
        path->push_back(make_shared<StateSpace2D> (StateSpace2D(x, y, orientation, vParam->maxVelocity)));
    }
    return path;
}

shared_ptr<geometry_msgs::Point> StateSpace2D::toPoint(void)
{
    geometry_msgs::Point p;
    p.x = x_;
    p.y = y_;
    return make_shared<geometry_msgs::Point>(p);
}