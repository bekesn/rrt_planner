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
    StateSpaceSimulated(float x, float y, float theta, float v = 4, float delta = 0); //TODO
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

    // Access variables
    float v(void);
    float delta(void);

};

#endif //STATESPACE_H