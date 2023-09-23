#include "MapHandler.h"

MapHandler::MapHandler(void)
{
    // Initialize values
    vehicleModel = make_shared<VehicleModel>();
    mapReceived = false;
    blueTrackBoundaryReceived = false;
    yellowTrackBoundaryReceived = false;
    goalState = make_shared<SS_VECTOR>();
    state = EMPTY;

}

MapHandler::MapHandler(unique_ptr<MAP_PARAMETERS> param, const shared_ptr<VehicleModel> vm) : MapHandler()
{
    mapParam = move(param);
    vehicleModel = vm;
}

bool MapHandler::isOffCourse(const shared_ptr<PATH_TYPE>& trajectory, const unique_ptr<RRT_PARAMETERS>& param) const
{
    switch (state)
    {
        case EMPTY:
            return false;
            break;
        case NOBOUNDARIES:
            return isOffCourseNoBoundary(trajectory, param);
            break;
        case BOUNDARIESGIVEN:
            return isOffCourseWithBoundary(trajectory, param);
            break;
    }
}

bool MapHandler::isOffCourseNoBoundary(const shared_ptr<PATH_TYPE>& trajectory, const unique_ptr<RRT_PARAMETERS>& param) const
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
        double dist = trajectory->front().getDistEuclidean((StateSpace2D){(float) cone->x, (float) cone->y, 0});
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
         if(isOnTrackEdge(make_shared<SS_VECTOR>(*it), closeBlueLandmarks, param) || isOnTrackEdge(make_shared<SS_VECTOR>(*it), closeYellowLandmarks, param))
         {
            isOC = true;
            break;
         }
    }

    delete closeBlueLandmarks;
    delete closeYellowLandmarks;

    return isOC;
}

bool MapHandler::isOffCourseWithBoundary(const shared_ptr<PATH_TYPE>& trajectory, const unique_ptr<RRT_PARAMETERS>& param) const
{
    // Filter empty trajectory
    if (trajectory->size() == 0) return false;

    //ROS_INFO_STREAM("cones: " << map.size());
    bool isOC;
    PATH_TYPE::iterator it;

    isOC = false;
    for (it = trajectory->begin(); it != trajectory->end(); it++)
    {   
        bool isOnEdge = false;
        shared_ptr<SS_VECTOR> vehicleState = make_shared<SS_VECTOR>(*it);
        int size = blueTrackBoundary.size();

        for(int i = 1; i < size; i++)
        {
            isOnEdge = isOnTrackEdge(vehicleState, blueTrackBoundary[i-1], blueTrackBoundary[i], param); 
            if (isOnEdge) 
            {
                isOC = true;
                break;
            }
        }

        size = yellowTrackBoundary.size();

        for(int i = 1; i < size; i++)
        {
            isOnEdge = isOnTrackEdge(vehicleState, yellowTrackBoundary[i-1], yellowTrackBoundary[i], param); 
            if (isOnEdge) 
            {
                isOC = true;
                break;
            }
        }

        if (isOC) break;
    }

    return isOC;
}

bool MapHandler::isOnTrackEdge(const shared_ptr<SS_VECTOR>& vehicleState, const std::vector<frt_custom_msgs::Landmark*>* cones, const unique_ptr<RRT_PARAMETERS>& param) const
{
    int size = cones->size();
    bool isOnEdge = false;

    for(int i = 0; i < size; i++)
    {
        for(int j = i+1; j < size; j++)
        {
            isOnEdge = isOnTrackEdge(vehicleState, ((*cones)[i]), ((*cones)[j]), param); 
            if (isOnEdge) break;
        }
    }
    return isOnEdge;
}

bool MapHandler::isOnTrackEdge(const shared_ptr<SS_VECTOR>& vehicleState, const frt_custom_msgs::Landmark* cone1, const frt_custom_msgs::Landmark* cone2 , const unique_ptr<RRT_PARAMETERS>& param) const
{
    float dx, dy, dx2, dy2, dist, projected, coneDist;
    float maxDist = vehicleModel->getParameters()->track / 2;
    bool isOnTrackEdge = false;

    dx = cone1->x - cone2->x;
    dy = cone1->y - cone2->y;
    dx2 = cone2->x - vehicleState->x();
    dy2 = cone2->y - vehicleState->y();
    coneDist = sqrt(dx*dx + dy*dy);
    if(coneDist < param->maxConeDist)
    {
        dist = abs(dx * dy2 - dx2 * dy) / coneDist;
        projected = (dx * (-dx2) + dy * (-dy2)) / coneDist;
        //ROS_INFO_STREAM("dist: " << dist << "   coneDist: " << coneDist << "   proj: " << projected << "   d: " << sqrt(dx2*dx2 + dy2*dy2));
        if((dist < maxDist) && (projected >= 0.0f) && (projected <= coneDist))
        {
            isOnTrackEdge = true;
        }
    }
    return isOnTrackEdge;
}

shared_ptr<SS_VECTOR> MapHandler::getRandomState(const shared_ptr<PATH_TYPE>& path, const unique_ptr<RRT_PARAMETERS>& param) const
{
    shared_ptr<SS_VECTOR> randState;
    double x, y, theta;
    int numOfCones = map.size();

    if ((path->size() > 0) && (((rand() % 1000) / 1000.0) > 0.5))
    {
        // Choose a state from best path randomly and place a state near it
        int nodeID = rand() % path->size();
        double range = param->sampleRange/10;

        x = (*path)[nodeID].x()+ (rand()%((int) (200*range))) / 100.0 - range;
        y  = (*path)[nodeID].y() + (rand()%((int) (200*range))) / 100.0 - range;
        theta = (rand() % ((int) (2000*M_PI))) / 1000.0 - M_PI;

        randState = make_shared<SS_VECTOR> (SS_VECTOR(x, y, theta));
    }
    else if ((((rand()%1000)/1000.0) > param->goalBias) && (numOfCones != 0)) 
    {
        // Choose a cone randomly and place a state near it
        int coneID = rand() % numOfCones;
        x = map[coneID]->x + (rand()%((int) (200*param->sampleRange))) / 100.0 - param->sampleRange;
        y = map[coneID]->y + (rand()%((int) (200*param->sampleRange))) / 100.0 - param->sampleRange;
        theta = (rand() % ((int) (2000*M_PI))) / 1000.0;

        randState = make_shared<SS_VECTOR> (SS_VECTOR(x, y, theta));
    }
    else
    {
        randState = goalState;
    }

    return randState;
}

unique_ptr<MAP_PARAMETERS>& MapHandler::getParameters(void)
{
    return mapParam;
}

void MapHandler::calculateGoalState()
{
    std::vector<std::vector<frt_custom_msgs::Landmark*>*> closestLandmarks;
    float maxDist = 0;
    shared_ptr<StateSpace2D> currentState = vehicleModel->getCurrentPose();

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

        SS_VECTOR state = SS_VECTOR(((*pair)[0]->x + (*pair)[1]->x) / 2, ((*pair)[0]->y + (*pair)[1]->y) / 2, 0);
        double angleDiff = abs(currentState->getAngleToTarget(state));
        if ((dist > maxDist) && (angleDiff < 1) && (dist < mapParam->goalHorizon))
        {
            maxDist = dist;
            goalState = make_shared<SS_VECTOR> (state);         
        }

        delete pair;
    }

}

shared_ptr<SS_VECTOR> MapHandler::getGoalState(void)
{
    return goalState;
}

void MapHandler::mapCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    mapReceived = updateLandmarkVector(msg, map);

    switch(state)
    {
        case EMPTY:
            if (yellowTrackBoundaryReceived && blueTrackBoundaryReceived)
            {
                state = BOUNDARIESGIVEN;
                calculateGoalState();
            }
            else if (mapReceived)
            {
                state = NOBOUNDARIES;
                calculateGoalState();
            }
            break;
        case NOBOUNDARIES:
            calculateGoalState();
            if (yellowTrackBoundaryReceived && blueTrackBoundaryReceived) state = BOUNDARIESGIVEN;
            break;
        case BOUNDARIESGIVEN:
            calculateGoalState(); //TODO temporary solution
            break;
    }
}

void MapHandler::blueTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    blueTrackBoundaryReceived =  updateLandmarkVector(msg, blueTrackBoundary);
}

void MapHandler::yellowTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    yellowTrackBoundaryReceived =  updateLandmarkVector(msg, yellowTrackBoundary);
}

bool MapHandler::updateLandmarkVector(const frt_custom_msgs::Map::ConstPtr &msg, vector<frt_custom_msgs::Landmark*> & vec)
{
    if (msg->map.empty())
    {
        return false;
    }

    for (int i = 0; i < vec.size(); i++) 
    {
        delete  vec[i];
    }
    vec.clear();


    for (frt_custom_msgs::Landmark landmark: msg->map) 
    {
        frt_custom_msgs::Landmark* newLandmark = new frt_custom_msgs::Landmark();
        newLandmark->x = landmark.x;
        newLandmark->y = landmark.y;
        newLandmark->color = landmark.color;
        vec.push_back(newLandmark);
    }

    return true;
}

void MapHandler::SLAMStatusCallback(const frt_custom_msgs::SlamStatus &msg)
{
    loopClosed = msg.loop_closed;
}

void MapHandler::visualizePoints(visualization_msgs::MarkerArray* mArray) const
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
    coord.x = goalState->x();
    coord.y = goalState->y();
    goal.points.push_back(coord);

    mArray->markers.emplace_back(goal);
}

frt_custom_msgs::Landmark* MapHandler::getClosestLandmark(const frt_custom_msgs::Landmark* selectedLandmark, const frt_custom_msgs::Landmark::_color_type color) const
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

MapHandlerState MapHandler::getState(void) const
{
    return state;
}

bool MapHandler::isLoopClosed(void) const
{
    return loopClosed;
}