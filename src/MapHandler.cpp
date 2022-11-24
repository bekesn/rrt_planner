#include "MapHandler.h"

MapHandler::MapHandler()
{
    goalBias = 0.2;
    vehicleModel = NULL;
    collisionRange = 6;
    spawnRange = 3;
}

MapHandler::MapHandler(VehicleModel* vm)
{
    goalBias = 0.2;
    vehicleModel = vm;
    collisionRange = 6;
    spawnRange = 3;
}


bool MapHandler::isOffCourse(std::vector<std::vector<double>>* trajectory)
{
    // Filter empty trajectory
    if (trajectory->size() == 0) return false;

    //ROS_INFO_STREAM("cones: " << map.size());
    bool isOC;
    std::vector<frt_custom_msgs::Landmark*>* closeBlueLandmarks = new std::vector<frt_custom_msgs::Landmark*>();
    std::vector<frt_custom_msgs::Landmark*>* closeYellowLandmarks = new std::vector<frt_custom_msgs::Landmark*>();
    std::vector<std::vector<double>>::iterator it;

    // Collect nearby cones with the same color
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
                    frt_custom_msgs::Landmark::_color_type col = getClosestLandmark(cone, frt_custom_msgs::Landmark::UNKNOWN)->color;
                    switch (col)
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
                    break;
            }
        }

        
    }

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
    double dx, dy, dx2, dy2, dist, projected, coneDist;
    int size = cones->size();
    float maxDist = vehicleModel->track / 2;
    bool isOnTrackEdge = false;

    for(int i = 0; i < size; i++)
    {
        for(int j = i+1; j < size; j++)
        {
            dx = (*cones)[i]->x - (*cones)[j]->x;
            dy = (*cones)[i]->y - (*cones)[j]->y;
            dx2 = (*cones)[j]->x - (*vehicleState)[0];
            dy2 = (*cones)[j]->y - (*vehicleState)[1];
            coneDist = sqrt(dx*dx + dy*dy);
            dist = abs(dx * dy2 - dx2 * dy) / coneDist;
            projected = (dx * (-dx2) + dy * (-dy2)) / coneDist;
            //ROS_INFO_STREAM("dist: " << dist << "   coneDist: " << coneDist << "   proj: " << projected << "   d: " << sqrt(dx2*dx2 + dy2*dy2));
            if((dist < maxDist) && (projected >= 0) && (projected <= coneDist))
            {
                isOnTrackEdge = true;
                break;
            }
        }
    }
    return isOnTrackEdge;
}

std::vector<double> MapHandler::getRandomState(std::vector<std::vector<double>>* path)
{
    std::vector<double> randState(3);


    if ((path->size() > 0) && (((rand() % 1000) / 1000.0) > 0.5))
    {
        int nodeID = rand() % path->size();
        double range = vehicleModel->getMaximalDistance();

        randState[0] = (*path)[nodeID][0] + (rand()%((int) (200*range))) / 100.0 - range;
        randState[1] = (*path)[nodeID][1] + (rand()%((int) (200*range))) / 100.0 - range;
        randState[2] = (rand() % ((int) (2000*M_PI))) / 1000.0 - M_PI;
    }
    else if (((rand()%1000)/1000.0) > goalBias) 
    {
        int numOfCones = map.size();
        if (numOfCones == 0)
        {
            randState = getGoalState();
        }
        else
        {
            // Choose a cone randomly and place a state near it
            int coneID = rand() % numOfCones;
            randState[0] = map[coneID]->x + (rand()%((int) (200*spawnRange))) / 100.0 - spawnRange;
            randState[1] = map[coneID]->y + (rand()%((int) (200*spawnRange))) / 100.0 - spawnRange;
            randState[2] = (rand() % ((int) (2000*M_PI))) / 1000.0;

        }
    }
    else
    {
        randState = getGoalState();
    }

    return randState;
}

void MapHandler::calculateGoalState()
{
    std::vector<std::vector<frt_custom_msgs::Landmark*>*> closestLandmarks;
    float maxDist = 0;
    std::vector<double> currentState = {vehicleModel->getCurrentPose().x, vehicleModel->getCurrentPose().y, vehicleModel->getCurrentPose().theta};
    


    // Create close blue-yellow pairs
    for (auto & landmark : map)
    {
        std::vector<frt_custom_msgs::Landmark*>* pair = new std::vector<frt_custom_msgs::Landmark*>;
        switch(landmark->color)
        {
            case frt_custom_msgs::Landmark::BLUE:
                pair->push_back(landmark);
                pair->push_back(getClosestLandmark(landmark, frt_custom_msgs::Landmark::YELLOW));
                break;
            case frt_custom_msgs::Landmark::YELLOW:
                pair->push_back(landmark);
                pair->push_back(getClosestLandmark(landmark, frt_custom_msgs::Landmark::BLUE));
                break;
            default:
                break;
        }
        if(pair->size() > 0)
        {
        }

        if((pair->size() > 0) && (vehicleModel->getDistEuclidean({landmark->x, landmark->y}, {(*pair)[1]->x, (*pair)[1]->y}) < 8))
        {
            closestLandmarks.push_back(pair);
        }
    }

    // Choose most distant pair

    for (auto & pair : closestLandmarks)
    {
        float dist = vehicleModel->getDistEuclidean(currentState, {(*pair)[0]->x, (*pair)[0]->y});
        dist += vehicleModel->getDistEuclidean(currentState, {(*pair)[1]->x, (*pair)[1]->y});
        dist = dist/2.0;

        std::vector<double> state = {((*pair)[0]->x + (*pair)[1]->x) / 2, ((*pair)[0]->y + (*pair)[1]->y) / 2};
        double angleDiff = abs((atan2((state[1] - currentState[1]), (state[0] - currentState[0])) - currentState[2]));
        /*ROS_INFO_STREAM("x1: " << (*pair)[0]->x << " y1: " << (*pair)[0]->y << " | x1: " << (*pair)[1]->x << " y1: " << (*pair)[1]->y << " dist: " << dist <<
        " angle:" << std::min(angleDiff, M_PI * 2.0 - angleDiff));*/
        if (dist > maxDist)
        {
            
            if (std::min(angleDiff, M_PI * 2.0 - angleDiff) < 1)
            {
                maxDist = dist;
                goalState = state;
            }
            
        }
    }

}


std::vector<double> MapHandler::getGoalState()
{
    return goalState;
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

    calculateGoalState();
}

void MapHandler::visualizePoints(visualization_msgs::MarkerArray* mArray)
{
    visualization_msgs::Marker goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.ns = "rrt_goal";
        goal.action = visualization_msgs::Marker::ADD;
        goal.pose.orientation.w = 1.0;
        goal.id = 3;
        goal.type = visualization_msgs::Marker::CUBE_LIST;
        goal.scale.x = 0.4f;
        goal.scale.y = 0.4f;
        goal.scale.z = 0.4f;
        goal.color.r = 1.0f;
        goal.color.g = 0.0f;
        goal.color.b = 0.0f;
        goal.color.a = 1.0f;

    geometry_msgs::Point coord;
    coord.x = goalState[0];
    coord.y = goalState[1];
    goal.points.push_back(coord);

    mArray->markers.emplace_back(goal);
}


frt_custom_msgs::Landmark* MapHandler::getClosestLandmark(frt_custom_msgs::Landmark* selectedLandmark, frt_custom_msgs::Landmark::_color_type color)
{
    float minDist = 100.0;
    float dist;
    frt_custom_msgs::Landmark* closestLandmark;
    bool colorOK;

    for(auto & landmark : map)
    {
        // Check if color is corresponding
        // If UNKNOWN, BLUE or YELLOW is accepted
        if (color != frt_custom_msgs::Landmark::UNKNOWN)
        {
            colorOK = landmark->color == color;
        }
        else
        {
            colorOK = (landmark->color == frt_custom_msgs::Landmark::BLUE) || (landmark->color == frt_custom_msgs::Landmark::YELLOW);
        }

        if ((landmark != selectedLandmark) && colorOK)
        {
            dist = vehicleModel->getDistEuclidean({landmark->x, landmark->y}, {selectedLandmark->x, selectedLandmark->y});
            if (dist < minDist)
            {
                closestLandmark = landmark;
                minDist = dist;
            }
        }
    }
    return closestLandmark;
}
