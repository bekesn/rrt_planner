#include "Control.h"
#include "StateSpaceSimulated.h"

unique_ptr<CONTROL_PARAMETERS> Control::controlParam;

Control::Control()
{

}

shared_ptr<Control> Control::control(const StateSpaceSimulated& state, const StateSpaceSimulated& target, const float& timeStep)
{
    shared_ptr<Control> input = shared_ptr<Control> (new Control());
    input->ddelta = psiLateralControl(state, target);
    input->ax = getRandomAccel();

    input->limitValues();
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

float Control::getRandomAccel(void)
{
    return (((float) (rand() % 100)) / 100.0f - 0.5f) * controlParam->maxLongAccel;
}

void Control::limitValues(void)
{
    if(ddelta > controlParam->maxdDelta) ddelta = controlParam->maxdDelta;
    else if(ddelta < -controlParam->maxdDelta) ddelta = -controlParam->maxdDelta;

    if(ax > controlParam->maxLongAccel) ax = controlParam->maxLongAccel;
    else if(ax < -controlParam->maxLongAccel) ax = -controlParam->maxLongAccel;
}