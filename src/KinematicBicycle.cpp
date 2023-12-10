#include "KinematicBicycle.h"
#include "Trajectory.h"
#include "Control.h"


KinematicBicycle::KinematicBicycle()
{
    x_ = 0;
    y_ = 0;
    theta_ = 0;
    vx_ = 1;
    delta_ = 0;
}

KinematicBicycle::KinematicBicycle(float x, float y, float theta, float vx, float delta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
    vx_ = vx;
    delta_ = delta;
}

KinematicBicycle::KinematicBicycle(const KinematicBicycle &original)
{
    x_ = original.x_;
    y_ = original.y_;
    theta_ = original.theta_;
    vx_ = original.vx_;
    delta_ = original.delta_;
}

void KinematicBicycle::initStateSpace(unique_ptr<CONTROL_PARAMETERS> controlParam)
{
    Control<KinematicBicycle>::setParameters(move(controlParam));
}

shared_ptr<KinematicBicycle> KinematicBicycle::derivative(const shared_ptr<Control<KinematicBicycle>>& controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const 
{
    float x, y, theta, v, delta;

    x = vx_ * cos(theta_);
    y = vx_ * sin(theta_);
    theta = vx_ / param->track * tan(delta_);
    v = controlInput->ax;
    delta = controlInput->ddelta;
    
    return shared_ptr<KinematicBicycle>(new KinematicBicycle(x, y, theta, v, delta));
}

bool KinematicBicycle::isGettingCloser(const shared_ptr<StateSpace2D> goalState, const unique_ptr<RRT_PARAMETERS>& rrtParam, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const
{
    // Calculation is based on derivative of getDistOriented
    // Derivative of state variables is calculated as in derivative(), but Control is not needed
    float dx, dy, dtheta;
    dx = vx_ * cos(theta_);
    dy = vx_ * sin(theta_);
    dtheta = vx_ / vehicleParam->track * tan(delta_);

    float numerator = (x_ - goalState->x()) * dx +
                      (y_ - goalState->y()) * dy ;//+
                      //(theta_ - goalState->theta()) * dtheta * rrtParam->thetaWeight;s
    return numerator < 0;
}

float KinematicBicycle::getDistOriented2(const StateSpace2D& otherState, const unique_ptr<RRT_PARAMETERS>& param) const
{
    float dx = x_ - otherState.x();
    float dy = y_ - otherState.y();
    float dtheta = getAngleDiff(otherState);
    return dx*dx + dy*dy + dtheta * dtheta * param->thetaWeight;
}

void KinematicBicycle::limitVariables(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    // Limit vx
    vx_ = min(max(vx_, 1.0f), vxLimit(vehicleParam));

    // Limit delta
    delta_ = min(max(delta_, -vehicleParam->maxDelta), vehicleParam->maxDelta);

    // Limit theta between -PI and PI
    theta_ = std::remainder(theta_, 2*M_PI);
}

float KinematicBicycle::vxLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const
{
    return vxLimitKinematic(vehicleParam, delta_);
}

float KinematicBicycle::vxLimitNext(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& ddelta, const float& timeStep) const
{
    return vxLimitKinematic(vehicleParam, delta_ + timeStep * ddelta);
}

float KinematicBicycle::vxLimitKinematic(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& delta) const
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

float KinematicBicycle::axLimit(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam) const
{
    float tmp = delta_ / (vehicleParam->wheelBase * vehicleParam->maxLatAccel);
    return vehicleParam->maxLongAccel * sqrt(1 - pow(vx_, 4) * tmp * tmp);
}

KinematicBicycle KinematicBicycle::operator+ (const KinematicBicycle & otherState) const
{
    return KinematicBicycle(x_ + otherState.x_, y_ + otherState.y_, theta_ + otherState.theta_, vx_ + otherState.vx_, delta_ + otherState.delta_);
}

KinematicBicycle KinematicBicycle::operator- (const KinematicBicycle & otherState) const
{
    return KinematicBicycle(x_ - otherState.x_, y_ - otherState.y_, theta_ - otherState.theta_, vx_ - otherState.vx_, delta_ - otherState.delta_);
}

KinematicBicycle KinematicBicycle::operator* (const float & multiplier) const
{
    return KinematicBicycle(x_ * multiplier, y_ * multiplier, theta_ * multiplier, vx_ * multiplier, delta_ * multiplier);
}

float KinematicBicycle::delta() const
{
    return delta_;
}


shared_ptr<Trajectory<KinematicBicycle>> KinematicBicycle::simulate(const shared_ptr<KinematicBicycle>& start, const shared_ptr<KinematicBicycle>& goal,
    const unique_ptr<RRT_PARAMETERS>& param,  const unique_ptr<VEHICLE_PARAMETERS>& vParam, const float& multiplier)
{
    shared_ptr<Trajectory<KinematicBicycle>> path = shared_ptr<Trajectory<KinematicBicycle>> (new Trajectory<KinematicBicycle>);
    shared_ptr<KinematicBicycle> state = start;
    state->limitVariables(vParam);

    bool gettingCloser = true;

    int numOfStates = (int) (multiplier * param->simulationTimeStep * state->vx() / param->resolution) + 1;
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
                shared_ptr<Control<KinematicBicycle>> controlInput = Control<KinematicBicycle>::control(*state, *goal, vParam, dt);
                state = RK4<KinematicBicycle>(state, controlInput, dt, vParam);
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
        shared_ptr<KinematicBicycle> prevState = path->back();
        bool closerFound = false;

        for (int i = 0; i < param->simIterations; i++)
        {
            dt = dt / 2.0f;
            
            // Simulate movement
            shared_ptr<Control<KinematicBicycle>> controlInput = Control<KinematicBicycle>::control(*state, *goal, vParam, dt);
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