#include "MapHandler.h"

template<class StateSpaceVector>
MapHandler<StateSpaceVector>::MapHandler(void)
{
    // Initialize values
    vehicle = make_shared<Vehicle<StateSpaceVector>>();
    mapReceived = false;
    blueTrackBoundaryReceived = false;
    yellowTrackBoundaryReceived = false;
    goalState = make_shared<StateSpaceVector>();
    state = EMPTY;

}

template<class StateSpaceVector>
MapHandler<StateSpaceVector>::MapHandler(unique_ptr<MAP_PARAMETERS> param, const shared_ptr<Vehicle<StateSpaceVector>> vm) : MapHandler()
{
    mapParam = move(param);
    vehicle = vm;
}

template<class StateSpaceVector>
bool MapHandler<StateSpaceVector>::isOffCourse(const shared_ptr<Trajectory<StateSpaceVector>>& trajectory) const
{
    if(mapKdTree->allnodes.size() < 2) return true;

    bool offCourse = false;
    for(auto state : *trajectory)
    {
        Kdtree::KdNodeVector knv;
        mapKdTree->range_nearest_neighbors({state->x(), state->y()}, mapParam->collisionRadius, &knv);
        if(knv.size() > 0)
        {
            offCourse = true;
            break;
        }
    }

    return offCourse;
}

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::upSample(void)
{
    Kdtree::KdNodeVector nodes, knv;
    frt_custom_msgs::Landmark::_color_type* color1, *color2;
    float x1, x2, y1, y2;
    int i, j;
    for(auto lm : map)
    {
        if(lm->color != frt_custom_msgs::Landmark::BLUE && lm->color != frt_custom_msgs::Landmark::YELLOW)
        {
            shared_ptr<frt_custom_msgs::Landmark> closest = getClosestLandmark(lm, frt_custom_msgs::Landmark::UNKNOWN);
            if(closest != NULL)
            {
                lm->color = closest->color;
                nodes.push_back(Kdtree::KdNode({lm->x, lm->y}, &closest->color));
            }
        }
        else
        {
            nodes.push_back(Kdtree::KdNode({lm->x, lm->y}, &lm->color));
        }
    }

    int size = nodes.size();
    if(size < 2) return;

    Kdtree::KdTree tmp = Kdtree::KdTree(&nodes);

    for(i = 0; i < size; i++)
    {   
        color1 = (frt_custom_msgs::Landmark::_color_type*) nodes[i].data;
        for(j = i+1; j < size; j++)
        {   
            color2 = (frt_custom_msgs::Landmark::_color_type*) nodes[j].data;
            if(*color1 == *color2)
            {
                x1 = nodes[i].point[0];
                y1 = nodes[i].point[1];
                x2 = nodes[j].point[0];
                y2 = nodes[j].point[1];
                float dist = StateSpace2D::getDistEuclidean({x1, y1},{x2, y2});
                tmp.range_nearest_neighbors({(x1 + x2)/2, (y1 + y2)/2}, dist/3, &knv);
                if((dist < mapParam->maxConeDist) && (dist > mapParam->maxGap) && (knv.size() == 0))
                {
                    int numOfGaps = ceil(dist / mapParam->maxGap);

                    for(int k = 1; k < numOfGaps; k++)
                    {
                        float x = (k * x1 + (numOfGaps - k) * x2)/((float) numOfGaps);
                        float y = (k * y1 + (numOfGaps - k) * y2)/((float) numOfGaps);

                        nodes.push_back(Kdtree::KdNode({x, y}, &map[i]->color));
                    }
                }
            }
        }
    }
    mapKdTree = shared_ptr<Kdtree::KdTree>(new Kdtree::KdTree(&nodes));
}

template<class StateSpaceVector>
shared_ptr<StateSpaceVector> MapHandler<StateSpaceVector>::getRandomState(const shared_ptr<Trajectory<StateSpaceVector>>& path, const unique_ptr<RRT_PARAMETERS>& param) const
{
    shared_ptr<StateSpaceVector> randState;
    double x, y, theta;
    int numOfCones = map.size();

    if ((path->size() > 0) && (((rand() % 1000) / 1000.0) > 0.5))
    {
        // Choose a state from best path randomly and place a state near it
        int nodeID = rand() % path->size();
        double range = param->sampleRange/10;

        x = (*path)[nodeID]->x() + (rand()%((int) (200*range))) / 100.0 - range;
        y = (*path)[nodeID]->y() + (rand()%((int) (200*range))) / 100.0 - range;
        theta = (rand() % ((int) (2000*M_PI))) / 1000.0 - M_PI;

        randState = make_shared<StateSpaceVector> (StateSpaceVector(x, y, theta, vehicle->getParameters()->maxVelocity));
    }
    else if ((((rand()%1000)/1000.0) > param->goalBias) && (numOfCones != 0)) 
    {
        // Choose a cone randomly and place a state near it
        int coneID = rand() % numOfCones;
        x = map[coneID]->x + (rand()%((int) (200*param->sampleRange))) / 100.0 - param->sampleRange;
        y = map[coneID]->y + (rand()%((int) (200*param->sampleRange))) / 100.0 - param->sampleRange;
        theta = (rand() % ((int) (2000*M_PI))) / 1000.0;

        randState = make_shared<StateSpaceVector> (StateSpaceVector(x, y, theta, vehicle->getParameters()->maxVelocity));
    }
    else
    {
        randState = goalState;
    }

    return randState;
}

template<class StateSpaceVector>
unique_ptr<MAP_PARAMETERS>& MapHandler<StateSpaceVector>::getParameters(void)
{
    return mapParam;
}

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::calculateGoalState()
{
    vector<shared_ptr<vector<shared_ptr<frt_custom_msgs::Landmark>>>> closestLandmarks;
    float maxDist2 = 0;
    shared_ptr<StateSpace2D> currentState = vehicle->getCurrentPose();

    // Create close blue-yellow pairs
    for (auto & landmark : map)
    {
        shared_ptr<vector<shared_ptr<frt_custom_msgs::Landmark>>> pair = shared_ptr<vector<shared_ptr<frt_custom_msgs::Landmark>>>(new vector<shared_ptr<frt_custom_msgs::Landmark>>);
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

        if((pair->size() > 0) && (StateSpace2D::getDistEuclidean2({(float) landmark->x, (float) landmark->y}, {(float) (*pair)[1]->x, (float) (*pair)[1]->y}) < mapParam->maxConeDist*mapParam->maxConeDist))
        {
            closestLandmarks.push_back(pair);
        }
    }

    // Choose most distant pair

    for (auto & pair : closestLandmarks)
    {
        float dist2 = currentState->getDistEuclidean2({(float) (*pair)[0]->x,(float) (*pair)[0]->y});
        dist2 += currentState->getDistEuclidean2({(float) (*pair)[1]->x, (float) (*pair)[1]->y});
        dist2 = dist2/2;

        StateSpaceVector state = StateSpaceVector(((*pair)[0]->x + (*pair)[1]->x) / 2, ((*pair)[0]->y + (*pair)[1]->y) / 2, 0,  vehicle->getParameters()->maxVelocity);
        double angleDiff = abs(currentState->getAngleToTarget(state));
        if ((dist2 > maxDist2) && (angleDiff < 1) && (dist2 < mapParam->goalHorizon*mapParam->goalHorizon))
        {
            maxDist2 = dist2;
            goalState = make_shared<StateSpaceVector> (state);         
        }
    }

}

template<class StateSpaceVector>
shared_ptr<StateSpaceVector> MapHandler<StateSpaceVector>::getGoalState(void)
{
    return goalState;
}

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::mapCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    mapReceived = updateLandmarkVector(msg, map);
    upSample();

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

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::blueTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    blueTrackBoundaryReceived =  updateLandmarkVector(msg, blueTrackBoundary);
}

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::yellowTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    yellowTrackBoundaryReceived =  updateLandmarkVector(msg, yellowTrackBoundary);
}

template<class StateSpaceVector>
bool MapHandler<StateSpaceVector>::updateLandmarkVector(const frt_custom_msgs::Map::ConstPtr &msg, vector<shared_ptr<frt_custom_msgs::Landmark>> & vec)
{
    if (msg->map.empty())
    {
        return false;
    }

    vec.clear();


    for (frt_custom_msgs::Landmark landmark: msg->map) 
    {
        vec.push_back(make_shared<frt_custom_msgs::Landmark>(landmark));
    }

    return true;
}

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::SLAMStatusCallback(const frt_custom_msgs::SlamStatus &msg)
{
    loopClosed = msg.loop_closed;
}

template<class StateSpaceVector>
void MapHandler<StateSpaceVector>::visualize(visualization_msgs::MarkerArray* mArray) const
{
    if(!mapReceived) return;

    // visualize goal state
    visualization_msgs::Marker goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.ns = "rrt_goal";
        goal.action = visualization_msgs::Marker::ADD;
        goal.pose.orientation.w = 1.0;
        goal.id = 3;
        goal.type = visualization_msgs::Marker::CUBE_LIST;
        goal.scale.x = 0.5f;
        goal.scale.y = 0.5f;
        goal.scale.z = 0.5f;
        goal.color.r = 1.0f;
        goal.color.g = 0.0f;
        goal.color.b = 0.0f;
        goal.color.a = 1.0f;

    goal.points.push_back(*goalState->toPoint());
    mArray->markers.emplace_back(goal);

    std_msgs::ColorRGBA varColor;
    varColor.r = 0;
    varColor.g = 0;
    varColor.b = 1;
    varColor.a = 1;

    // visualize upsampled map
    visualization_msgs::Marker upsampled;
        upsampled.header.frame_id = "map";
        upsampled.header.stamp = ros::Time::now();
        upsampled.ns = "rrt_map_extended";
        upsampled.action = visualization_msgs::Marker::ADD;
        upsampled.pose.orientation.w = 1.0;
        upsampled.id = 4;
        upsampled.type = visualization_msgs::Marker::SPHERE_LIST;
        upsampled.scale.x = 2*mapParam->collisionRadius;
        upsampled.scale.y = 2*mapParam->collisionRadius;
        upsampled.scale.z = 2*mapParam->collisionRadius;

    geometry_msgs::Point coord;
    for(auto node : mapKdTree->allnodes)
    {
        coord.x = node.point[0];
        coord.y = node.point[1];
        upsampled.points.push_back(coord);
        
        frt_custom_msgs::Landmark::_color_type* c = (frt_custom_msgs::Landmark::_color_type*) node.data;
        if(*c == frt_custom_msgs::Landmark::Type::BLUE)
        {
            varColor.r = 0;
            varColor.g = 0;
            varColor.b = 1;
        }
        else if(*c == frt_custom_msgs::Landmark::Type::YELLOW)
        {
            varColor.r = 1;
            varColor.g = 0.9;
            varColor.b = 0.3;
        }
        else
        {
            varColor.r = 0;
            varColor.g = 1;
            varColor.b = 0;
        }
        upsampled.colors.push_back(varColor);

    }
    mArray->markers.emplace_back(upsampled);
}

template<class StateSpaceVector>
shared_ptr<frt_custom_msgs::Landmark> MapHandler<StateSpaceVector>::getClosestLandmark(const shared_ptr<frt_custom_msgs::Landmark>& selectedLandmark, const frt_custom_msgs::Landmark::_color_type& color) const
{
    float minDist2 = 1000000.0;
    float dist;
    shared_ptr<frt_custom_msgs::Landmark> closestLandmark;
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
            dist = StateSpace2D::getDistEuclidean2({(float) landmark->x, (float) landmark->y}, {(float) selectedLandmark->x, (float) selectedLandmark->y});
            if (dist < minDist2)
            {
                closestLandmark = landmark;
                minDist2 = dist;
            }
        }
    }
    return closestLandmark;
}

template<class StateSpaceVector>
MapHandlerState MapHandler<StateSpaceVector>::getState(void) const
{
    return state;
}

template<class StateSpaceVector>
bool MapHandler<StateSpaceVector>::isLoopClosed(void) const
{
    return loopClosed;
}

// Define classes
template class MapHandler<StateSpace2D>;
template class MapHandler<KinematicBicycle>;
template class MapHandler<DynamicBicycle>;