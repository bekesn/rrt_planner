#ifndef STATESPACESIMULATED_H
#define STATESPACESIMULATED_H

#include "StateSpace2D.h"
#include "Control.h"

class StateSpaceSimulated : public StateSpace2D
{
    float v_;
    float delta_;
public:
    StateSpaceSimulated();
    StateSpaceSimulated(float x, float y, float theta, float v = 0, float delta = 0);
    StateSpaceSimulated(const StateSpaceSimulated &original);
    //~StateSpace();

    // Calculate derivative of statespace vector
    StateSpaceSimulated* derivative(Control* controlInput, VEHICLE_PARAMETERS* param);

    // Limit constrained state variables
    void limitVariables(RRT_PARAMETERS* rrtParam);
    
    // Override operators
    StateSpaceSimulated operator+ (const StateSpaceSimulated & otherState) const;
    StateSpaceSimulated operator- (const StateSpaceSimulated & otherState) const;
    StateSpaceSimulated operator* (const float & multiplier) const;

};

#endif //STATESPACE_H