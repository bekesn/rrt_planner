#include "MapHandler.h"

MapHandler::MapHandler()
{
    goalBias = 0.2;
    vehicleModel = NULL;
    collisionRange = 6;
}

MapHandler::MapHandler(VehicleModel* vm)
{
    goalBias = 0.2;
    vehicleModel = vm;
    collisionRange = 6;
}


bool MapHandler::isOffCourse(std::vector<std::vector<double>>* trajectory)
{
    ROS_INFO_STREAM("cones: " << map.size());
    bool isOC;
    std::vector<frt_custom_msgs::Landmark*>* closeBlueLandmarks = new std::vector<frt_custom_msgs::Landmark*>();
    std::vector<frt_custom_msgs::Landmark*>* closeYellowLandmarks = new std::vector<frt_custom_msgs::Landmark*>();
    std::vector<std::vector<double>>::iterator it;

    for (auto & cone : map)
    {
        double dist = vehicleModel->getDistEuclidean(trajectory->front(), {cone->x, cone->y, 0});
        if (dist < collisionRange)
        {
            switch (cone->color)
            {
                case frt_custom_msgs::Landmark::BLUE:
                    closeBlueLandmarks->push_back(cone);
                    break;
                case frt_custom_msgs::Landmark::YELLOW:
                    closeYellowLandmarks->push_back(cone);
                    break;
                default:
                    break;
            }
        }

        
    }
    //TODO
    ROS_INFO_STREAM("close: " << closeBlueLandmarks->size() << " blue, " << closeYellowLandmarks->size() << " yellow");

    isOC = false;
    for (it = trajectory->begin(); it != trajectory->end(); it++)
    {
         if(isOnTrackEdge(&(*it), closeBlueLandmarks) || isOnTrackEdge(&(*it), closeYellowLandmarks))
         {
            isOC = true;
            break;
         }
    }

    free(closeBlueLandmarks);
    free(closeYellowLandmarks);

    return isOC;
}


bool MapHandler::isOnTrackEdge(std::vector<double>* vehicleState, std::vector<frt_custom_msgs::Landmark*>* cones)
{
    double dx, dy, dist;
    int size = cones->size();
    float maxDist = vehicleModel->track / 2;
    bool isOnTrackEdge = false;

    for(int i = 0; i < size; i++)
    {
        for(int j = i+1; j < size; j++)
        {
            dx = (*cones)[i]->x - (*cones)[j]->x;
            dy = (*cones)[i]->y - (*cones)[j]->y;
            dist = abs(dx*((*cones)[j]->y - (*vehicleState)[1]) - ((*cones)[j]->x - (*vehicleState)[0])*dy) / sqrt(dx*dx + dy*dy);
            if(dist < maxDist)
            {
                isOnTrackEdge = true;
                break;
            }
        }
    }
    return isOnTrackEdge;
}

std::vector<double> MapHandler::getRandomState()
{
    std::vector<double> randState = calculateGoalState();

    if (((rand()%1000)/1000.0) > goalBias)
    {
        //TODO adaptive boundaries
        randState[0] = rand()%10000 / 100.0 - 50;
        randState[1] = rand()%10000 / 100.0 - 50;
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