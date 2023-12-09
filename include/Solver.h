#ifndef SOLVER_H
#define SOLVER_H

#include "StateSpace2D.h"

template<class StateSpaceVector>
class Control;
class KinematicBicycle;
class DynamicBicycle;

// Runge-Kutta 4th order
template<class StateSpaceVector>
shared_ptr<StateSpaceVector> RK4(const shared_ptr<StateSpaceVector>& startState,
                                 const shared_ptr<Control<StateSpaceVector>>& controlInput, float dt, const unique_ptr<VEHICLE_PARAMETERS>& vParam);

#endif /*SOLVER_H*/