#include "Solver.h"
#include "DynamicBicycle.h"

template<class StateSpaceVector>
shared_ptr<StateSpaceVector> RK4(const shared_ptr<StateSpaceVector>& startState,
                                 const shared_ptr<Control<StateSpaceVector>>& controlInput, float dt, const unique_ptr<VEHICLE_PARAMETERS>& vParam)
{
    StateSpaceVector k1, k2, k3, k4, k;
    k1 = *(startState->derivative(controlInput, vParam)) * dt;
    k2 = *startState + k1*0.5;
    k2 = *(k2.derivative(controlInput, vParam)) * dt;
    k3 = *startState + k2*0.5;
    k3 = *(k3.derivative(controlInput, vParam)) * dt;
    k4 = *startState + k3;
    k4 = *(k4.derivative(controlInput, vParam)) * dt;
    k = (k1 + k2*2 + k3*2 + k4) * (1.0f/6.0f);

    return make_shared<StateSpaceVector>((*startState) + k);
}

// Define functions
template shared_ptr<KinematicBicycle> RK4(const shared_ptr<KinematicBicycle>&, const shared_ptr<Control<KinematicBicycle>>& , float, const unique_ptr<VEHICLE_PARAMETERS>&);
template shared_ptr<DynamicBicycle> RK4(const shared_ptr<DynamicBicycle>&, const shared_ptr<Control<DynamicBicycle>>& , float, const unique_ptr<VEHICLE_PARAMETERS>&);
