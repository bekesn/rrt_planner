#include "MapHandler.h"

MapHandler::MapHandler()
{
    // Initialize values
    vehicleModel = NULL;
    mapReceived = false;
    goalState = StateSpace2D();

}

MapHandler::MapHandler(MAP_PARAMETERS* param, VehicleModel* vm) : MapHandler()
{
    mapParam = param;
    vehicleModel = vm;
}

bool MapHandler::isOffCourse(PATH_TYPE* trajectory, RRT_PARAMETERS* param)
{
    // Filter empty trajectory
    if (trajectory->size() == 0) return false;

    //ROS_INFO_STREAM("cones: " << map.size());
    bool isOC;
    std::vector<frt_custom_msgs::Landmark*>* closeBlueLandmarks = new std::vector<frt_custom_msgs::Landmark*>();
    std::vector<frt_custom_msgs::Landmark*>* closeYellowLandmarks = new std::vector<frt_custom_msgs::Landmark*>();
    PATH_TYPE::iterator it;

    // Collect nearby cones with the same color
    for (auto & cone : map)
    {
        double dist = trajectory->front().getDistEuclidean({(float) cone->x, (float) cone->y, 0});
        if (dist < param->collisionRange)
        {
            switch (cone->color)
            {
                frt_custom_msgs::Landmark::_color_type col;
                
                case frt_custom_msgs::Landmark::BLUE:
                    closeBlueLandmarks->push_back(cone);
                    break;
                case frt_custom_msgs::Landmark::YELLOW:
                    closeYellowLandmarks->push_back(cone);
                    break;
                case frt_custom_msgs::Landmark::ORANGE_BIG:
                    col = getClosestLandmark(cone, frt_custom_msgs::Landmark::UNKNOWN)->color;
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
                default:
                    break;
            }
        }
    }

    isOC = false;
    for (it = trajectory->begin(); it != trajectory->end(); it++)
    {
         if(isOnTrackEdge(&(*it), closeBlueLandmarks, param) || isOnTrackEdge(&(*it), closeYellowLandmarks, param))
         {
            isOC = true;
            break;
         }
    }

    delete closeBlueLandmarks;
    delete closeYellowLandmarks;

    return isOC;
}

bool MapHandler::isOnTrackEdge(SS_VECTOR* vehicleState, std::vector<frt_custom_msgs::Landmark*>* cones, RRT_PARAMETERS* param)
{
    double dx, dy, dx2, dy2, dist, projected, coneDist;
    int size = cones->size();
    float maxDist = vehicleModel->getParameters()->track / 2;
    bool isOnTrackEdge = false;

    for(int i = 0; i < size; i++)
    {
        for(int j = i+1; j < size; j++)
        {
            dx = (*cones)[i]->x - (*cones)[j]->x;
            dy = (*cones)[i]->y - (*cones)[j]->y;
            dx2 = (*cones)[j]->x - vehicleState->x();
            dy2 = (*cones)[j]->y - vehicleState->y();
            coneDist = sqrt(dx*dx + dy*dy);
            if(coneDist < param->maxConeDist)
            {
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
    }
    return isOnTrackEdge;
}

SS_VECTOR MapHandler::getRandomState(PATH_TYPE* path, RRT_PARAMETERS* param)
{
    SS_VECTOR randState;
    double x, y, theta;


    if ((path->size() > 0) && (((rand() % 1000) / 1000.0) > 0.5))
    {
        int nodeID = rand() % path->size();
        double range = param->sampleRange/10;

        x = (*path)[nodeID].x()+ (rand()%((int) (200*range))) / 100.0 - range;
        y  = (*path)[nodeID].y() + (rand()%((int) (200*range))) / 100.0 - range;
        theta = (rand() % ((int) (2000*M_PI))) / 1000.0 - M_PI;

        randState = SS_VECTOR(x, y, theta);
    }
    else if (((rand()%1000)/1000.0) > param->goalBias) 
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
            x = map[coneID]->x + (rand()%((int) (200*param->sampleRange))) / 100.0 - param->sampleRange;
            y = map[coneID]->y + (rand()%((int) (200*param->sampleRange))) / 100.0 - param->sampleRange;
            theta = (rand() % ((int) (2000*M_PI))) / 1000.0;

            randState = SS_VECTOR(x, y, theta);
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
    SS_VECTOR* currentState = vehicleModel->getCurrentPose();
    


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

        if((pair->size() > 0) && (StateSpace2D::getDistEuclidean({(float) landmark->x, (float) landmark->y}, {(float) (*pair)[1]->x, (float) (*pair)[1]->y}) < 8))
        {
            closestLandmarks.push_back(pair);
        }
    }

    // Choose most distant pair

    for (auto & pair : closestLandmarks)
    {
        float dist = currentState->getDistEuclidean({(float) (*pair)[0]->x,(float) (*pair)[0]->y});
        dist += currentState->getDistEuclidean({(float) (*pair)[1]->x, (float) (*pair)[1]->y});
        dist = dist/2.0;

        SS_VECTOR state = StateSpace2D(((*pair)[0]->x + (*pair)[1]->x) / 2, ((*pair)[0]->y + (*pair)[1]->y) / 2, 0);
        double angleDiff = abs(currentState->angleToTarget(&state));
        if ((dist > maxDist) && (angleDiff < 1) && (dist < mapParam->goalHorizon))
        {
            maxDist = dist;
            goalState = state;         
        }

        delete pair;
    }

}

SS_VECTOR MapHandler::getGoalState()
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

    mapReceived = true;
    calculateGoalState();
}

void MapHandler::visualizePoints(visualization_msgs::MarkerArray* mArray)
{
    if(!mapReceived);

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
    coord.x = goalState.x();
    coord.y = goalState.y();
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
            dist = StateSpace2D::getDistEuclidean({(float) landmark->x, (float) landmark->y}, {(float) selectedLandmark->x, (float) selectedLandmark->y});
            if (dist < minDist)
            {
                closestLandmark = landmark;
                minDist = dist;
            }
        }
    }
    return closestLandmark;
}

bool MapHandler::hasMap()
{
    return mapReceived;
}