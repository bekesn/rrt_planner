#include "Control.h"

template<class StateSpaceVector>
unique_ptr<CONTROL_PARAMETERS> Control<StateSpaceVector>::controlParam;

template<class StateSpaceVector>
Control<StateSpaceVector>::Control()
{

}

template<class StateSpaceVector>
shared_ptr<Control<StateSpaceVector>> Control<StateSpaceVector>::control(const StateSpaceVector& state, const StateSpaceVector& target,
                                     const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep)
{
    float delta;
    shared_ptr<Control> input = shared_ptr<Control> (new Control());

    delta = psiLateralControl(state, target, vehicleParam);
    input->ddelta = controlParam->kDelta*(delta - state.delta());
    input->ax = longitudinalControl(state, target, timeStep);

    input->limitValues(state, vehicleParam, timeStep);
    return input;
}

template<class StateSpaceVector>
float Control<StateSpaceVector>::thetaLateralControl(const StateSpace2D& state, const StateSpace2D& target, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    return 0;
}

template<class StateSpaceVector>
float Control<StateSpaceVector>::psiLateralControl(const StateSpace2D& state, const StateSpace2D& target, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    float psi = state.getAngleToTarget(target);
    float delta = controlParam->kPsi * psi;

    delta = min(max(delta, -vehicleParam->maxDelta), vehicleParam->maxDelta);

    return delta;
}

template<class StateSpaceVector>
float Control<StateSpaceVector>::longitudinalControl(const StateSpaceVector& state, const StateSpaceVector& target, const float& timeStep)
{
    return (target.vx() - state.vx()) / timeStep;
}

template<class StateSpaceVector>
void Control<StateSpaceVector>::setParameters(unique_ptr<CONTROL_PARAMETERS> param)
{
    controlParam = move(param);
}

template<class StateSpaceVector>
unique_ptr<CONTROL_PARAMETERS>& Control<StateSpaceVector>::getParameters(void)
{
    return controlParam;
}

template<class StateSpaceVector>
float Control<StateSpaceVector>::getRandomAccel(const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam)
{
    return (((float) (rand() % 100)) / 50.0f - 1.0f) * vehicleParam->maxLongAccel;
}

template<class StateSpaceVector>
void Control<StateSpaceVector>::limitValues(const StateSpaceVector& state, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam, const float& timeStep)
{
    ddelta = min(max(ddelta, -vehicleParam->maxdDelta), vehicleParam->maxdDelta);

    float axLimit, axPred;
    axLimit = state.axLimit(vehicleParam);
    axPred = (state.vxLimitNext(vehicleParam, ddelta, timeStep) - state.vx()) / timeStep;

    ax = min(max(ax, -axLimit), min(axLimit, axPred));
}

template class Control<KinematicBicycle>;
template class Control<DynamicBicycle>;
