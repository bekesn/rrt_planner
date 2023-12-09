#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <visualization_msgs/Marker.h>
#include "Types.h"
#include "DynamicBicycle.h"

template<typename StateSpaceVector>
class Trajectory : public vector<shared_ptr<StateSpaceVector>>
{

    
public:

    using vector<shared_ptr<StateSpaceVector>>::push_back;

    Trajectory(/* args */);
    ~Trajectory();

    // COST FUNCTIONS
    // Cost function
    double cost(const unique_ptr<RRT_PARAMETERS>& param) const;

    // Cost according to length of trajectory
    double getDistanceCost(void) const;

    // Cost according to elapsed time
    double getTimeCost(void) const;

    // Visualize
    void visualize(visualization_msgs::Marker* marker);

};


#endif //TRAJECTORY_H