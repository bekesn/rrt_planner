#include "Control.h"
#include "StateSpaceSimulated.h"


Control::Control()
{

}

Control* Control::stanleyToTarget(SS_VECTOR* state, StateSpace2D* target)
{

}

Control* Control::angleControl(SS_VECTOR* state, StateSpace2D* target)
{
    float angle = state->getAngleToTarget(target);
    Control* input = new Control();
    input->ddelta = (-controlParam->k * angle) - state->delta();
    input->ax = getRandomAccel();
    input->MYaw = 0;

    input->limitValues();
    return input;
}

void Control::setParam(CONTROL_PARAMETERS* param)
{
    controlParam = param;
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