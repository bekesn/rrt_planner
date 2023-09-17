#include "Control.h"
#include "StateSpaceSimulated.h"

unique_ptr<CONTROL_PARAMETERS> Control::controlParam;

Control::Control()
{

}

Control* Control::stanleyToTarget(const SS_VECTOR* state, const StateSpace2D* target)
{

}

Control* Control::angleControl(const SS_VECTOR* state, const StateSpace2D* target)
{
    float angle = state->getAngleToTarget(target);
    Control* input = new Control();
    input->ddelta = (-controlParam->k * angle) - state->delta();
    input->ax = getRandomAccel();
    input->MYaw = 0;

    input->limitValues();
    return input;
}

void Control::setParameters(unique_ptr<CONTROL_PARAMETERS> param)
{
    controlParam = move(param);
}

unique_ptr<CONTROL_PARAMETERS>& Control::getParameters(void)
{
    return controlParam;
}

double Control::getRandomAccel(void)
{
    return (((float) (rand() % 100)) / 100.0f - 0.5f) * controlParam->maxLongAccel;
}

void Control::limitValues(void)
{
    if(ddelta > controlParam->maxdDelta) ddelta = controlParam->maxdDelta;
    else if(ddelta < -controlParam->maxdDelta) ddelta = -controlParam->maxdDelta;
}