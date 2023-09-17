#ifndef STATESPACE2D_H
#define STATESPACE2D_H

#include <math.h>
#include <geometry_msgs/Pose2D.h>
#include "Types.h"
#include <cereal/archives/xml.hpp>


class StateSpace2D
{
protected:
    float x_;
    float y_;
    float theta_;

public:
    StateSpace2D();
    StateSpace2D(float x, float y, float theta);
    StateSpace2D(const StateSpace2D &original);
    //~StateSpace2D();

    // Calculate distance to target, taking into account the orientation
    float getDistToTarget(const StateSpace2D* target, const RRT_PARAMETERS* param) const;

    // Calculate distance between states, taking into account the orientation
    static float getDistOriented(const StateSpace2D* state1, const StateSpace2D* state2, const RRT_PARAMETERS* param);
    float getDistOriented(const StateSpace2D* otherState, const RRT_PARAMETERS* param) const;

    // Euclidean distance
    static float getDistEuclidean(const std::vector<float> state1, const std::vector<float> state2);
    float getDistEuclidean(const std::vector<float> otherState) const;
    float getDistEuclidean(const StateSpace2D* otherState) const;

    // Calculate angular difference in rad
    // Anticlockwise
    float getAngleToTarget(const StateSpace2D* target) const;
    float getAngleDiff(const StateSpace2D* otherState) const;

    // Define operators
    StateSpace2D operator+ (const StateSpace2D & otherState) const;
    StateSpace2D operator- (const StateSpace2D & otherState) const;
    StateSpace2D operator* (const float & multiplier) const;

    float x(void) const;
    float y(void) const;
    float theta(void) const;
    
    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(CEREAL_NVP(x_), CEREAL_NVP(y_), CEREAL_NVP(theta_));}   
};

#endif //STATESPACE2D_H