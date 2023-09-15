#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Types.h"
#include "StateSpaceSimulated.h"
#include <vector>

class Trajectory : public std::vector<SS_VECTOR>
{
    
public:
    Trajectory(/* args */);
    ~Trajectory();

    // COST FUNCTIONS
    // Cost function
    double cost(const RRT_PARAMETERS* param) const;

    // Cost according to length of trajectory
    double getDistanceCost(void) const;

    // Cost according to elapsed time
    double getTimeCost(void) const;

};


#endif //TRAJECTORY_H