#include "MapHandler.h"

MapHandler::MapHandler()
{
    goalBias = 0.05;
}

bool MapHandler::isOffCourse(std::vector<std::vector<double>>* path)
{
    ROS_INFO_STREAM("cones: " << map.size());

    
    return false;
}

std::vector<double> MapHandler::getRandomState()
{
    std::vector<double> randState = calculateGoalState();

    if (((rand()%1000)/1000.0) > goalBias)
    {
        randState[0] = rand()%5000 / 100.0 - 5;
        randState[1] = rand()%5000 / 100.0 - 5;
    }

    return randState;
}

std::vector<double> MapHandler::calculateGoalState()
{
    return std::vector<double> {75.0, 0, 0};
}

void MapHandler::mapCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    if (msg->map.empty())
    {
        return;
    }

    for (int i = 0; i < map.size(); i++) 
    {
        delete  map[i];
    }
    map.clear();


    for (frt_custom_msgs::Landmark landmark: msg->map) 
    {
        frt_custom_msgs::Landmark* newLandmark = new frt_custom_msgs::Landmark();
        newLandmark->x = landmark.x;
        newLandmark->y = landmark.y;
        newLandmark->color = landmark.color;
        this->map.push_back(newLandmark);
    }
}