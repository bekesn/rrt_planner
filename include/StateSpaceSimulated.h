#ifndef STATESPACESIMULATED_H
#define STATESPACESIMULATED_H

#include "StateSpace2D.h"
#include "Control.h"
#include <cereal/types/base_class.hpp>

class StateSpaceSimulated : public StateSpace2D
{
    float v_;
    float delta_;
public:
    StateSpaceSimulated();
    StateSpaceSimulated(float x, float y, float theta, float v = 10, float delta = 0); //TODO
    StateSpaceSimulated(const StateSpaceSimulated &original);
    //~StateSpace();

    // Calculate derivative of statespace vector
    StateSpaceSimulated* derivative(const Control* controlInput, const unique_ptr<VEHICLE_PARAMETERS>& param) const;

    // Limit constrained state variables
    void limitVariables(const unique_ptr<RRT_PARAMETERS>& rrtParam, const unique_ptr<VEHICLE_PARAMETERS>& vehicleParam);
    
    // Override operators
    StateSpaceSimulated operator+ (const StateSpaceSimulated & otherState) const;
    StateSpaceSimulated operator- (const StateSpaceSimulated & otherState) const;
    StateSpaceSimulated operator* (const float & multiplier) const;

    // Access variables
    float v(void) const ;
    float delta(void) const ;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::base_class<StateSpace2D> ( this ), CEREAL_NVP(v_), CEREAL_NVP(delta_));}

};

#endif //STATESPACE_H