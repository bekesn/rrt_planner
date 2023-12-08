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

bool MapHandler::isOffCourse(const shared_ptr<PATH_TYPE>& trajectory) const
{
    if(mapKdTree->allnodes.size() < 2) return true;

    bool offCourse = false;
    for(auto state : *trajectory)
    {
        Kdtree::KdNodeVector knv;
        mapKdTree->range_nearest_neighbors({state->x(), state->y()}, 0.7, &knv);
        if(knv.size() > 0)
        {
            offCourse = true;
            break;
        }
    }

    return offCourse;
}

void MapHandler::upSample(void)
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

        x = (*path)[nodeID]->x() + (rand()%((int) (200*range))) / 100.0 - range;
        y = (*path)[nodeID]->y() + (rand()%((int) (200*range))) / 100.0 - range;
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
    vector<shared_ptr<vector<shared_ptr<frt_custom_msgs::Landmark>>>> closestLandmarks;
    float maxDist = 0;
    shared_ptr<StateSpace2D> currentState = vehicleModel->getCurrentPose();

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
    }

}

shared_ptr<SS_VECTOR> MapHandler::getGoalState(void)
{
    return goalState;
}

void MapHandler::mapCallback(const frt_custom_msgs::Map::ConstPtr &msg)
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

void MapHandler::blueTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    blueTrackBoundaryReceived =  updateLandmarkVector(msg, blueTrackBoundary);
}

void MapHandler::yellowTrackBoundaryCallback(const frt_custom_msgs::Map::ConstPtr &msg)
{
    yellowTrackBoundaryReceived =  updateLandmarkVector(msg, yellowTrackBoundary);
}

bool MapHandler::updateLandmarkVector(const frt_custom_msgs::Map::ConstPtr &msg, vector<shared_ptr<frt_custom_msgs::Landmark>> & vec)
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

void MapHandler::SLAMStatusCallback(const frt_custom_msgs::SlamStatus &msg)
{
    loopClosed = msg.loop_closed;
}

void MapHandler::visualize(visualization_msgs::MarkerArray* mArray) const
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
        goal.scale.x = 0.2f;
        goal.scale.y = 0.2f;
        goal.scale.z = 0.2f;
        goal.color.r = 1.0f;
        goal.color.g = 0.0f;
        goal.color.b = 0.0f;
        goal.color.a = 1.0f;

    geometry_msgs::Point coord;
    coord.x = goalState->x();
    coord.y = goalState->y();
    goal.points.push_back(coord);

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
        upsampled.ns = "rrt_map";
        upsampled.action = visualization_msgs::Marker::ADD;
        upsampled.pose.orientation.w = 1.0;
        upsampled.id = 4;
        upsampled.type = visualization_msgs::Marker::CUBE_LIST;
        upsampled.scale.x = 0.4f;
        upsampled.scale.y = 0.4f;
        upsampled.scale.z = 0.4f;
        /*upsampled.color.r = 1.0f;
        upsampled.color.g = 1.0f;
        upsampled.color.b = 0.0f;
        upsampled.color.a = 1.0f;*/

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

shared_ptr<frt_custom_msgs::Landmark> MapHandler::getClosestLandmark(const shared_ptr<frt_custom_msgs::Landmark>& selectedLandmark, const frt_custom_msgs::Landmark::_color_type& color) const
{
    float minDist = 1000.0;
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