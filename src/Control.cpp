#include "Control.h"
#include "StateSpaceSimulated.h"

unique_ptr<CONTROL_PARAMETERS> Control::controlParam;

Control::Control()
{

}

shared_ptr<Control> Control::control(const StateSpaceSimulated& state, const StateSpaceSimulated& target,
                                     const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep)
{
    shared_ptr<Control> input = shared_ptr<Control> (new Control());
    input->ddelta = psiLateralControl(state, target);
    input->ax = getRandomAccel(vehicleParam);

    input->limitValues(state, vehicleParam, timeStep);
    return input;
}

float Control::thetaLateralControl(const StateSpace2D& state, const StateSpace2D& target)
{
    return 0;
}

float Control::psiLateralControl(const StateSpace2D& state, const StateSpace2D& target)
{
    float psi = state.getAngleToTarget(target);
    float ddelta = controlParam->k * psi;

    return ddelta;
}


float Control::longitudinalControl(const StateSpaceSimulated& state, const StateSpaceSimulated& target, const float& timeStep)
{
    float ax;

    ax = (target.v() - state.v()) / timeStep;
    return 0;
}

void Control::setParameters(unique_ptr<CONTROL_PARAMETERS> param)
{
    controlParam = move(param);
}

unique_ptr<CONTROL_PARAMETERS>& Control::getParameters(void)
{
    return controlParam;
}

float Control::getRandomAccel(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    return (((float) (rand() % 100)) / 50.0f - 1.0f) * vehicleParam->maxLongAccel;
}

void Control::limitValues(const StateSpaceSimulated& state, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep)
{
    if(ddelta > vehicleParam->maxdDelta) ddelta = vehicleParam->maxdDelta;
    else if(ddelta < -vehicleParam->maxdDelta) ddelta = -vehicleParam->maxdDelta;

    float axMax, axLimit, axPred;
    axLimit = state.axLimit(vehicleParam);
    axPred = (state.vxLimitNext(vehicleParam, ddelta, timeStep) - state.v()) / timeStep;
    axMax = min(axLimit, axPred);

    if(ax > axMax) ax = axMax;
    else if(ax < -axLimit) ax = -axLimit;
}