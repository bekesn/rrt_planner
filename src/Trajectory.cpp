#include "Trajectory.h"


Trajectory::Trajectory(/* args */)
{
}

Trajectory::~Trajectory()
{
}

double Trajectory::cost(const unique_ptr<RRT_PARAMETERS>& param) const
{
    double cost;
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

double Trajectory::getDistanceCost(void) const
{
    if(this->size() < 2) return 100;
    
    SS_VECTOR prevState = (*this)[0];
    SS_VECTOR currState;
    int size = this->size();
    double length = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*this)[i];
        length += prevState.getDistEuclidean(&currState);
        prevState = currState;
    }
    return length;
}

double Trajectory::getTimeCost(void) const
{
    if(this->size() < 2) return 100;
    
    SS_VECTOR prevState = (*this)[0];
    SS_VECTOR currState;
    int size = this->size();
    double elapsed = 0;
    for (int i = 1; i < size; i++)
    {
        currState = (*this)[i];
        if(prevState.v() > 0)
        {
            elapsed += prevState.getDistEuclidean(&currState) / prevState.v();
        }
        else
        {
            elapsed += 100;
        }
        prevState = currState;
    }
    return elapsed;
}
