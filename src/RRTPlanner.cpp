#include "RRTPlanner.h"


template<class StateSpaceVector>
RRTPlanner<StateSpaceVector>::RRTPlanner(int argc, char** argv)
{
    ros::NodeHandle nh;
    nodeName = "[" + ros::this_node::getName() + "]";

    state = NOMAP;

    // Init parameter structs
    genParam = unique_ptr<GENERAL_PARAMETERS> (new GENERAL_PARAMETERS);
    unique_ptr<VEHICLE_PARAMETERS> vehicleParam = unique_ptr<VEHICLE_PARAMETERS> (new VEHICLE_PARAMETERS);
    unique_ptr<MAP_PARAMETERS> mapParam = unique_ptr<MAP_PARAMETERS> (new MAP_PARAMETERS);
    unique_ptr<CONTROL_PARAMETERS> controlParam = unique_ptr<CONTROL_PARAMETERS> (new CONTROL_PARAMETERS);

    // Init objects with zero parameters
    vehicle = make_shared<Vehicle<StateSpaceVector>> (Vehicle<StateSpaceVector>(move(vehicleParam)));
    mapHandler = make_shared<MapHandler<StateSpaceVector>> (MapHandler<StateSpaceVector>(move(mapParam), vehicle));
    localRRT = unique_ptr<SearchTree<StateSpaceVector>>(new SearchTree<StateSpaceVector>(make_shared<StateSpaceVector> (StateSpaceVector()), LOCAL_RRT));
    globalRRT = unique_ptr<SearchTree<StateSpaceVector>>(new SearchTree<StateSpaceVector>(make_shared<StateSpaceVector> (StateSpaceVector()), GLOBAL_RRT));

    // Load ROS parameters
    loadParameters(controlParam);

    StateSpaceVector::initStateSpace(move(controlParam));

    // Subscribe to map
    ROS_INFO_STREAM("" << nodeName << " Node started.");
    mapSubscriber = nh.subscribe("/map", 1, &MapHandler<StateSpaceVector>::mapCallback, &(*(mapHandler.get())));
    blueTrackBoundarySubscriber = nh.subscribe("/blueTrackBoundary", 1, &MapHandler<StateSpaceVector>::blueTrackBoundaryCallback, &(*(mapHandler.get())));
    yellowTrackBoundarySubscriber = nh.subscribe("/yellowTrackBoundary", 1, &MapHandler<StateSpaceVector>::yellowTrackBoundaryCallback, &(*(mapHandler.get())));
    poseSubscriber = nh.subscribe("/pose", 1, &Vehicle<StateSpaceVector>::poseCallback, &(*(vehicle)));
    SLAMStatusSubscriber = nh.subscribe("/slam_status", 1, &MapHandler<StateSpaceVector>::SLAMStatusCallback, &(*(mapHandler.get())));
    odometrySubscriber = nh.subscribe("/odometry/velocity", 1, &Vehicle<StateSpaceVector>::velocityCallback, &(*(vehicle)));
    localRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_local_viz", 10);
    globalRRT->markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_global_viz", 10);
    commonPublisher = nh.advertise<visualization_msgs::MarkerArray>("/rrt_common_viz", 10);

    timer = nh.createWallTimer(ros::WallDuration(genParam->timerPeriod), &RRTPlanner::timerCallback, this);
    
    ros::spin();
}


template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::stateMachine()
{
    switch(state)
    {
        case NOMAP:
            if (mapHandler->getState() != EMPTY) state = LOCALPLANNING;
            break;
        case LOCALPLANNING:
            planLocalRRT();
            if (mapHandler->isLoopClosed() && handleActualPath())
            {
                state = WAITFORGLOBAL;
            }
            break;
        case WAITFORGLOBAL:
            planLocalRRT();
            planGlobalRRT();
            if(globalRRT->pathFound) state = GLOBALPLANNING;
            break;
        case GLOBALPLANNING:
            planGlobalRRT();
            break;
    }
}


template<class StateSpaceVector>
shared_ptr<SearchTreeNode<StateSpaceVector>> RRTPlanner<StateSpaceVector>::extend(unique_ptr<SearchTree<StateSpaceVector>>& rrt)
{
    shared_ptr<SearchTreeNode<StateSpaceVector>> newNode;
    bool offCourse, alreadyInTree;
    shared_ptr<StateSpaceVector> randState;
    shared_ptr<StateSpaceVector> newState;
    shared_ptr<Trajectory<StateSpaceVector>> trajectory;

    // Get random state
    randState = mapHandler->getRandomState(rrt->getBestPath(), rrt->param);

    // Get nearest node
    shared_ptr<SearchTreeNode<StateSpaceVector>> nearest = rrt->getNearest(randState);

    // Simulate movement towards new state
    trajectory = StateSpaceVector::simulate(nearest->getState(), randState, rrt->param, vehicle->getParameters());

    // Check for offCourse
    offCourse = mapHandler->isOffCourse(trajectory);

    // Add node
    if ((!offCourse) && (trajectory->size() > 1))
    {
        // Check if new node would be duplicate
        newState = trajectory->back();
        alreadyInTree = rrt->alreadyInTree(newState);

        if(!alreadyInTree)
        {
            newNode = rrt->addChild(nearest, newState, trajectory->getTimeCost());
        }
    }
    else
    {
        newNode = NULL;
    }

    return newNode;
}

template<class StateSpaceVector>
bool RRTPlanner<StateSpaceVector>::rewire(unique_ptr<SearchTree<StateSpaceVector>>& rrt, shared_ptr<SearchTreeNode<StateSpaceVector>> newNode)
{
    shared_ptr<Trajectory<StateSpaceVector>> trajectory;
    shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> nearbyNodes = rrt->getNearby(newNode);
    typename vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>::iterator it;
    float newNodeCost = rrt->getAbsCost(newNode);
    bool offCourse;

    for (it = nearbyNodes->begin(); it != nearbyNodes->end(); it++)
    {
        
        trajectory = StateSpaceVector::simulate(newNode->getState(), (*it)->getState(), rrt->param, vehicle->getParameters(), rrt->param->rewireTime);
        offCourse = mapHandler->isOffCourse(trajectory);
        if ((trajectory->size() > 1) && !offCourse)
        {
            float distError2 = trajectory->back()->getDistOriented2(*(*it)->getState(), rrt->param);
            // Check if new path leads close to new state
            if ( distError2 < rrt->param->minDeviation * rrt->param->minDeviation)
            {
                float segmentCost = trajectory->getTimeCost();
                float childCost = rrt->getAbsCost(*it);
                float timeError = sqrt(distError2) / trajectory->back()->vx();
                // Compare costs  if it is worth rewiring
                if ((newNodeCost + segmentCost + timeError) < childCost)
                {
                    //Rewire if it reduces cost
                    rrt->rewire(*it,newNode);
                    (*it)->changeSegmentCost(segmentCost);
                }
                else if ((newNodeCost - globalRRT->param->minCost) > childCost)
                {
                    // If newNodeCost is significantly higher than childCost, a loop might be created
                    // Rewiring would create a loop with multiple problems
                    // Instead of making a loop, mark it in the SearchTree
                    bool isLoop = rrt->addLoop(newNode, *it, segmentCost);

                    // If a loop is not created, rewire 
                    if(!isLoop)
                    {
                        rrt->rewire(*it,newNode);
                        (*it)->changeSegmentCost(segmentCost);
                    }
                }
            }
        }
    }
    return false;
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::optimizeTriangles(unique_ptr<SearchTree<StateSpaceVector>>& rrt)
{
    for(int i = 0; i < rrt->param->triangleIterations; i++)
    {
        shared_ptr<SearchTreeNode<StateSpaceVector>> node = rrt->getRandomNode();
        optimizeTriangle(rrt, node);
    }
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::optimizeTriangle(unique_ptr<SearchTree<StateSpaceVector>>& rrt, shared_ptr<SearchTreeNode<StateSpaceVector>> node)
{
    shared_ptr<SearchTreeNode<StateSpaceVector>> parent, parentParent;
    shared_ptr<Trajectory<StateSpaceVector>> trajectory;
    bool isClose, isLowerCost, offCourse;
    float segmentCost;

    parent = node->getParent();
    if(parent == NULL) return;
    parentParent = parent->getParent();
    if(parentParent == NULL) return;

    trajectory = StateSpaceVector::simulate(parentParent->getState(), node->getState(), rrt->param, vehicle->getParameters(), rrt->param->rewireTime);
    if(trajectory->size() > 0)
    {
        segmentCost = trajectory->getTimeCost();
        shared_ptr<StateSpace2D> s = node->getState();
        float error2 = trajectory->back()->getDistOriented2(*s, rrt->param);
        isClose =  error2 < rrt->param->minDeviation;
        isLowerCost = segmentCost < parent->getSegmentCost() + node->getSegmentCost() + rrt->param->minDeviation / (parentParent->getState()->vx() *2);
        offCourse = mapHandler->isOffCourse(trajectory);

        if(isClose && isLowerCost && !offCourse)
        {
            rrt->rewire(node, parentParent);
            node->changeSegmentCost(segmentCost);
            if(parent->getChildren()->size() == 0)
            {
                rrt->remove(parent);
            }

        }
    }

}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::planLocalRRT(void)
{
    shared_ptr<StateSpaceVector> pose = vehicle->getCurrentPose();
    localRRT->init(pose);
    int iteration = 0;
    shared_ptr<StateSpaceVector> goalState = mapHandler->getGoalState();

    while((!localRRT->maxNumOfNodesReached()) && (iteration <= localRRT->param->iterations))
    {
        shared_ptr<SearchTreeNode<StateSpaceVector>> node = extend(localRRT);
        if (node != NULL)
        {   
            bool rewired = rewire(localRRT, node);
            bool isClose = node->getState()->getDistEuclidean2(*goalState) < localRRT->param->goalRadius*localRRT->param->goalRadius;
            if ((localRRT->pathFound && rewired) || isClose)
            {
                localRRT->updatePath(localRRT->traceBackToRoot(localRRT->getNearest(goalState)));
            }

            if(isClose)
            {
                localRRT->pathFound = true;
            }
        }
        optimizeTriangles(localRRT);

        iteration++;
    }

    if(localRRT->pathFound)
    {
        localRRT->updatePath(localRRT->traceBackToRoot(localRRT->getNearest(goalState))->getSimulated(localRRT->param, vehicle->getParameters()));
    }

}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::planGlobalRRT(void)
{
    int iteration = 0;
    bool changed = false;
    while((!globalRRT->maxNumOfNodesReached()) && (iteration <= globalRRT->param->iterations))
    {
        shared_ptr<SearchTreeNode<StateSpaceVector>> node = extend(globalRRT);
        if (node != NULL)
        {   
            bool rewired = rewire(globalRRT, node);
            if(rewired) changed = true;
        }

        optimizeTriangles(globalRRT);
        
        iteration++;
    }

    globalRRT->manageLoops(vehicle->getParameters());
}

template<class StateSpaceVector>
bool RRTPlanner<StateSpaceVector>::handleActualPath(void)
{
    shared_ptr<Trajectory<StateSpaceVector>> actualPath = vehicle->getActualPath();
    float fullCost = actualPath->getTimeCost();
    if (fullCost < globalRRT->param->minCost) return false;

    shared_ptr<Trajectory<StateSpaceVector>> loop = shared_ptr<Trajectory<StateSpaceVector>> (new Trajectory<StateSpaceVector>);
    shared_ptr<StateSpaceVector> currentPose = actualPath->back();
    typename Trajectory<StateSpaceVector>::iterator it;
    bool isLoop = false;
    float cost = 0;
    Trajectory<StateSpaceVector> segment;
    segment.push_back(actualPath->front());

    float distStep = globalRRT->param->simulationTimeStep * currentPose->vx();

    for(it = actualPath->begin()+1; it != actualPath->end(); it++)
    {
        if ((currentPose->getDistToTarget2(**it, globalRRT->param) < distStep*distStep) &&
            (cost < (fullCost - 3 * globalRRT->param->rewireTime * globalRRT->param->simulationTimeStep)))
        {
            isLoop = true;
        }
        if (isLoop)
        {
            loop->push_back(*it);
        }

        // Calculate cost
        segment.push_back(*it);
        cost += segment.getTimeCost();

        // Reset segment
        segment.erase(segment.begin());
    }

    if(isLoop)
    {
        actualPath = loop;
        globalRRT->init(loop);
    }

    return isLoop;
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::timerCallback(const ros::WallTimerEvent &event)
{
    ros::Time startTime = ros::Time::now();
    stateMachine();
    visualize();
    ros::Duration runTime = ros::Time::now() - startTime;
    if(runTime.toSec() > genParam->timerPeriod)
    {
         ROS_WARN_STREAM("[" << ros::this_node::getName() << "] Runtime high: " << ((int) (runTime.toSec()*1000)) << " ms");
    }
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::visualize(void)
{
    localRRT->markerArray.markers.clear();
    localRRT->visualize();
    localRRT->markerPublisher.publish(localRRT->markerArray);

    globalRRT->markerArray.markers.clear();
    globalRRT->visualize();
    globalRRT->markerPublisher.publish(globalRRT->markerArray);

    commonMArray.markers.clear();
    mapHandler->visualize(&commonMArray);
    vehicle->visualize(&commonMArray);
    commonPublisher.publish(commonMArray);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner");

    string simType;
    while(!ros::param::get("/rrt_planner/VEHICLE/simType", simType))
    {
        sleep(10);
    }
    if (simType == "HOLONOMIC")
    {
        RRTPlanner<StateSpace2D> planner(argc, argv);
    }
    else if (simType == "KINEMATIC")
    {
        RRTPlanner<KinematicBicycle> planner(argc, argv);
    }
    else if (simType == "DYNAMIC")
    {
        RRTPlanner<DynamicBicycle> planner(argc, argv);
    }
    else
    {
        ROS_ERROR_STREAM("[rrt_planner] Wrong simType parameter. Using holonomic model.");
        RRTPlanner<StateSpace2D> planner(argc, argv);
    }

    return 0;
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::loadParameter(const string& topic, float& parameter, const float defaultValue)
{
    if (!ros::param::get("/rrt_planner/" + topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  std::to_string(parameter)); 
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::loadParameter(const string& topic, int& parameter, const int defaultValue)
{
    if (!ros::param::get("/rrt_planner/" + topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  std::to_string(parameter)); 
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::loadParameter(const string& topic, string& parameter, const string defaultValue)
{
    if (!ros::param::get("/rrt_planner/" + topic, parameter))
    {
        parameter = defaultValue;
        ROS_ERROR_STREAM(nodeName << " Parameter " << topic << " not found."); 
    }
    ROS_INFO_STREAM(nodeName << " " << topic << "  =  " <<  parameter);     
}

template<class StateSpaceVector>
void RRTPlanner<StateSpaceVector>::loadParameters(unique_ptr<CONTROL_PARAMETERS>& controlParam)
{
    ROS_INFO_STREAM(nodeName << " LOADING PARAMETERS");
    loadParameter("/GENERAL/timerPeriod", genParam->timerPeriod, 0.1f);

    // Choose distance calculation type
    std::string costType;
    loadParameter("/LOCAL/costType", costType, "DISTANCE");
    if (costType == "DISTANCE") localRRT->param->costType = DISTANCE;
    else if (costType == "TIME") localRRT->param->costType = TIME;

    loadParameter("/GLOBAL/costType", costType, "DISTANCE");
    if (costType == "DISTANCE") globalRRT->param->costType = DISTANCE;
    else if (costType == "TIME") globalRRT->param->costType = TIME;

    loadParameter("/LOCAL/goalBias", localRRT->param->goalBias, 0.2f);
    loadParameter("/GLOBAL/goalBias", globalRRT->param->goalBias, 0.2f);

    loadParameter("/LOCAL/goalRadius", localRRT->param->goalRadius, 1.0f);
    loadParameter("/GLOBAL/goalRadius", globalRRT->param->goalRadius, 1.0f);

    loadParameter("/LOCAL/iterations", localRRT->param->iterations, 500);
    loadParameter("/GLOBAL/iterations", globalRRT->param->iterations, 500);

    loadParameter("/LOCAL/maxNumOfNodes", localRRT->param->maxNumOfNodes, 1000);
    loadParameter("/GLOBAL/maxNumOfNodes", globalRRT->param->maxNumOfNodes, 1000);

    loadParameter("/LOCAL/minCost", localRRT->param->minCost, 0.0f);
    loadParameter("/GLOBAL/minCost", globalRRT->param->minCost, 3.0f);

    loadParameter("/LOCAL/minDeviation", localRRT->param->minDeviation, 0.1f);
    loadParameter("/GLOBAL/minDeviation", globalRRT->param->minDeviation, 0.1f);

    loadParameter("/LOCAL/minDeviationDist", localRRT->param->minDeviationDist, 0.05f);
    loadParameter("/GLOBAL/minDeviationDist", globalRRT->param->minDeviationDist, 0.05f);

    loadParameter("/LOCAL/psiWeight", localRRT->param->psiWeight, 5000.0f);
    loadParameter("/GLOBAL/psiWeight", globalRRT->param->psiWeight, 5000.0f);

    loadParameter("/LOCAL/resolution", localRRT->param->resolution, 0.05f);
    loadParameter("/GLOBAL/resolution", globalRRT->param->resolution, 0.05f);

    loadParameter("/LOCAL/rewireTime", localRRT->param->rewireTime, 3.0f);
    loadParameter("/GLOBAL/rewireTime", globalRRT->param->rewireTime, 3.0f);

    loadParameter("/LOCAL/sampleRange", localRRT->param->sampleRange, 3);
    loadParameter("/GLOBAL/sampleRange", globalRRT->param->sampleRange, 3);

    loadParameter("/LOCAL/simIterations", localRRT->param->simIterations, 1);
    loadParameter("/GLOBAL/simIterations", globalRRT->param->simIterations, 1);

    loadParameter("/LOCAL/simulationTimeStep", localRRT->param->simulationTimeStep, 0.2f);
    loadParameter("/GLOBAL/simulationTimeStep", globalRRT->param->simulationTimeStep, 0.2f);

    loadParameter("/LOCAL/thetaWeight", localRRT->param->thetaWeight, 50.0f);
    loadParameter("/GLOBAL/thetaWeight", globalRRT->param->thetaWeight, 50.0f);

    loadParameter("/LOCAL/triangleIterations", localRRT->param->triangleIterations, 1.0f);
    loadParameter("/GLOBAL/triangleIterations", globalRRT->param->triangleIterations, 1.0f);

    loadParameter("/VEHICLE/maxdDelta", vehicle->getParameters()->maxdDelta, 0.1f);
    loadParameter("/VEHICLE/maxDelta", vehicle->getParameters()->maxDelta, 0.38f);
    loadParameter("/VEHICLE/maxLatAccel", vehicle->getParameters()->maxLatAccel, 10.0f);
    loadParameter("/VEHICLE/maxLongAccel", vehicle->getParameters()->maxLongAccel, 5.0f);
    loadParameter("/VEHICLE/maxVelocity", vehicle->getParameters()->maxVelocity, 5.0f);
    loadParameter("/VEHICLE/track", vehicle->getParameters()->track, 1.2f);
    loadParameter("/VEHICLE/wheelBase", vehicle->getParameters()->wheelBase, 1.54f); 

    loadParameter("/MAP/collisionRange", mapHandler->getParameters()->collisionRange, 6.0f);
    loadParameter("/MAP/goalHorizon", mapHandler->getParameters()->goalHorizon, 15.0f);
    loadParameter("/MAP/maxConeDist", mapHandler->getParameters()->maxConeDist, 6.0f);
    loadParameter("/MAP/maxGap", mapHandler->getParameters()->maxGap, 1.5f);

    loadParameter("/CONTROL/k", controlParam->k, 15.0f);

    // Choose simulation type
    std::string simType;
    loadParameter("/VEHICLE/simType", simType, "HOLONOMIC");
    if (simType == "HOLONOMIC") vehicle->getParameters()->simType = HOLONOMIC;
    else if (simType == "KINEMATIC") vehicle->getParameters()->simType = KINEMATIC;
    else if (simType == "DYNAMIC") vehicle->getParameters()->simType = DYNAMIC;
}

// Define classes
template class RRTPlanner<StateSpace2D>;
template class RRTPlanner<KinematicBicycle>;
template class RRTPlanner<DynamicBicycle>;