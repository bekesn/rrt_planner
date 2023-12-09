#include "Trajectory.h"


template<class StateSpaceVector>
Trajectory<StateSpaceVector>::Trajectory(/* args */)
{
}

template<class StateSpaceVector>
Trajectory<StateSpaceVector>::~Trajectory()
{
};

template<class StateSpaceVector>
float Trajectory<StateSpaceVector>::cost(const unique_ptr<RRT_PARAMETERS>& param) const
{
    float cost;
    switch(param->costType)
    {
        case DISTANCE:
            cost = this->getDistanceCost();
            break;
        case TIME:
            cost = this->getTimeCost();
            break;
        default:
            throw std::invalid_argument("Wrong cost calculation type");
            break;
    }

    return cost;
}

template<class StateSpaceVector>
float Trajectory<StateSpaceVector>::getDistanceCost(void) const
{
    if(this->size() < 2) return 100;
    
    shared_ptr<StateSpace2D> prevState = (*this)[0];
    shared_ptr<StateSpace2D> currState;
    int size = this->size();
    float length = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*this)[i];
        length += prevState->getDistEuclidean(*currState);
        prevState = currState;
    }
    return length;
}

template<class StateSpaceVector>
float Trajectory<StateSpaceVector>::getTimeCost(void) const
{
    if(this->size() < 2) return 100;
    
    shared_ptr<StateSpace2D> prevState = (*this)[0];
    shared_ptr<StateSpace2D> currState;
    int size = this->size();
    float elapsed = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*this)[i];
        if(prevState->vx() > 0)
        {
            elapsed += prevState->getDistEuclidean(*currState) / prevState->vx();
        }
        else
        {
            elapsed += 100;
        }
        prevState = currState;
    }
    return elapsed;
}

template<class StateSpaceVector>
shared_ptr<Trajectory<StateSpaceVector>> Trajectory<StateSpaceVector>::getSimulated(const unique_ptr<RRT_PARAMETERS>& param, const unique_ptr<VEHICLE_PARAMETERS>& vParam)
{
    int size = this->size();
    shared_ptr<Trajectory<StateSpaceVector>> simulated = shared_ptr<Trajectory<StateSpaceVector>> (new Trajectory<StateSpaceVector>);
    shared_ptr<Trajectory<StateSpaceVector>> tmp;
    for(int i = 1; i < size; i++)
    {
        tmp = StateSpaceVector::simulate((*this)[i-1], (*this)[i], param, vParam);
        tmp->pop_back();
        simulated->insert(simulated->end(), make_move_iterator(tmp->begin()), make_move_iterator(tmp->end()));
    }
    return simulated;
}

template<class StateSpaceVector>
void Trajectory<StateSpaceVector>::visualize(visualization_msgs::Marker* marker)
{
    typename Trajectory<StateSpaceVector>::iterator pathIterator;
    for (pathIterator = this->begin(); pathIterator != this->end(); pathIterator++)
    {
        marker->points.push_back(*(*pathIterator)->toPoint());
    }
}

// Define classes
template class Trajectory<StateSpace2D>;
template class Trajectory<KinematicBicycle>;
template class Trajectory<DynamicBicycle>;