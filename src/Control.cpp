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
    float angle = state->angleToTarget(target);
    Control* input = new Control();
    input->ddelta = (-controlParam->k * angle) - state->delta();
    input->dv = 0;
    input->MYaw = 0;

    input->limitValues();
    return input;
}

void Control::setParam(CONTROL_PARAMETERS* param)
{
    controlParam = param;
}

void Control::limitValues(void)
{
    if(ddelta > controlParam->maxdDelta) ddelta = controlParam->maxdDelta;
    else if(ddelta < -controlParam->maxdDelta) ddelta = -controlParam->maxdDelta;
}