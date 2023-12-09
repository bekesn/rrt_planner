#include <SearchTree.h>


template<class StateSpaceVector>
SearchTree<StateSpaceVector>::SearchTree()
{
    param = unique_ptr<RRT_PARAMETERS> (new RRT_PARAMETERS);
    type = LOCAL_RRT;
    bestPath = shared_ptr<Trajectory<StateSpaceVector>> (new Trajectory<StateSpaceVector>);
    tree = unique_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> (new vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>);

    pathLength = 0;
    pathTime = 0;
    pathCost = 0;
}

template<class StateSpaceVector>
SearchTree<StateSpaceVector>::SearchTree(shared_ptr<StateSpaceVector> startState, RRT_TYPE rrtType) : SearchTree()
{
    type = rrtType;

    this->init(startState);
}

template<class StateSpaceVector>
SearchTree<StateSpaceVector>::~SearchTree()
{

}

template<class StateSpaceVector>
shared_ptr<SearchTreeNode<StateSpaceVector>> SearchTree<StateSpaceVector>::addChild(shared_ptr<SearchTreeNode<StateSpaceVector>> parentNode, shared_ptr<StateSpaceVector> state, double nodeCost)
{
    if (!maxNumOfNodesReached())
    {
        tree->push_back(make_shared<SearchTreeNode<StateSpaceVector>>(SearchTreeNode<StateSpaceVector>(parentNode, state, nodeCost)));
        parentNode->addChild(tree->back());
        nodeCount++;
        return tree->back();
    }
    else
    {
        ROS_WARN("[RRT_PLANNER]: MAX NUMBER OF NODES REACHED");
        return NULL;
    }
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::remove(shared_ptr<SearchTreeNode<StateSpaceVector>> node)
{
    int numOfChildren = node->getChildren()->size();
    if(numOfChildren == 0)
    {   
        // Do not delete from vEdges
        bool isVEdge = false;
        for(auto vEdge : vEdges)
        {
            if(vEdge->start == node)
            {
                isVEdge = true;
                break;
            }
        }
        if(!isVEdge)
        {
            node->getParent()->removeChild(node);
            tree->erase(std::remove(tree->begin(), tree->end(), node),tree->end());
            nodeCount--;
        }
    }
    else
    {
        ROS_WARN_STREAM("Trying to delete node with " << numOfChildren << " children.");
    }
}

template<class StateSpaceVector>
shared_ptr<SearchTreeNode<StateSpaceVector>> SearchTree<StateSpaceVector>::getNearest(const shared_ptr<StateSpaceVector>& toState, float minCost) const
{
    typename vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>::iterator it;
    double minDist2;
    double dist2;
    shared_ptr<SearchTreeNode<StateSpaceVector>> closest;

    // Initialize closest node and distance
    minDist2 = tree->front()->getState()->getDistToTarget2(*toState, param);
    closest = tree->front();

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if(getAbsCost(*it) >= minCost)
        {
            dist2 = (*it)->getState()->getDistToTarget2(*toState, param);
            if (dist2 < minDist2)
            {
                minDist2 = dist2;
                closest = (*it);
            }
        }
    }

    if(getAbsCost(closest) < minCost) closest = NULL;

    return closest;
}

template<class StateSpaceVector>
shared_ptr<std::vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> SearchTree<StateSpaceVector>::getNearby(shared_ptr<SearchTreeNode<StateSpaceVector>> node) const
{
    typename vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>::iterator it;
    auto closeNodes = shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> (new vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>);
    shared_ptr<StateSpaceVector> state = node->getState();
    float range2 = param->rewireTime * param->simulationTimeStep * state->vx();
    range2 = range2 * range2;

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if ((state->getDistToTarget2(*(*it)->getState(), param) < range2) && ((*it) != node))
        {
            closeNodes->push_back((*it));
        }
    }

    return closeNodes;

}

template<class StateSpaceVector>
bool SearchTree<StateSpaceVector>::alreadyInTree(const shared_ptr<StateSpaceVector>& state) const
{
    typename std::vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>::iterator it;
    bool isInTree = false;
    float minDeviationDist2 = param->minDeviationDist * param->minDeviationDist;
    float minDeviation2 = param->minDeviation * param->minDeviation;

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if(state->getDistEuclidean2(*(*it)->getState()) < minDeviationDist2)
        {
           isInTree = true;
           break;
        }
        else if(state->getDistOriented2(*(*it)->getState(), param) < minDeviation2)
        {
           isInTree = true;
           break;
        }
    }

    return isInTree;
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::visualize(void)
{
    if(tree->size() < 2) return;

    float relativeVelocity;
    std_msgs::ColorRGBA varColor;
    varColor.r = 0;
    varColor.g = 1;
    varColor.b = 0;
    varColor.a = 1;


    // RRT nodes visualization
    visualization_msgs::Marker treeNodes;
        treeNodes.header.frame_id = "map";
        treeNodes.header.stamp = ros::Time::now();
        treeNodes.ns = "rrt_states";
        treeNodes.action = visualization_msgs::Marker::ADD;
        treeNodes.pose.orientation.w = 1.0;
        treeNodes.id = 0;
        treeNodes.type = visualization_msgs::Marker::CUBE_LIST;
        treeNodes.scale.x = 0.15f;
        treeNodes.scale.y = 0.15f;
        treeNodes.scale.z = 0.15f;

    geometry_msgs::Point coord;
    
    typename vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>::iterator it;
    for (it = tree->begin(); it != tree->end(); it++)
    {
        treeNodes.points.push_back(*(*it)->getState()->toPoint());

        relativeVelocity = (*it)->getState()->vx() / 10;
        if(relativeVelocity > 1.0f) relativeVelocity = 1.0f;
        varColor.r = relativeVelocity;
        varColor.g = 1 - relativeVelocity;
        treeNodes.colors.push_back(varColor);
    }

    markerArray.markers.emplace_back(treeNodes);

    // RRT edges visualization
    visualization_msgs::Marker graphEdge;
        graphEdge.header.frame_id = "map";
        graphEdge.header.stamp = ros::Time::now();
        graphEdge.ns = "rrt_graph_edge";
        graphEdge.action = visualization_msgs::Marker::ADD;
        graphEdge.pose.orientation.w = 1.0;
        graphEdge.id = 1;
        graphEdge.type = visualization_msgs::Marker::LINE_LIST;
        graphEdge.scale.x = 0.05f;
        graphEdge.color.r = 1.0f;
        graphEdge.color.g = 0.65f;
        graphEdge.color.b = 0.0f;
        graphEdge.color.a = 1.0f;

    typename vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>::iterator treeIterator;

    for (treeIterator = tree->begin()+1; treeIterator != tree->end(); treeIterator++)
    {
            graphEdge.points.push_back(*(*treeIterator)->getParent()->getState()->toPoint());
            graphEdge.points.push_back(*(*treeIterator)->getState()->toPoint());
    }

    markerArray.markers.emplace_back(graphEdge);
    
    if (this->pathFound && (bestPath->size() > 1))
    {        
        visualization_msgs::Marker pathLine;
            pathLine.header.frame_id = "map";
            pathLine.header.stamp = ros::Time::now();
            pathLine.ns = "rrt_best_path";
            pathLine.action = visualization_msgs::Marker::ADD;
            pathLine.pose.orientation.w = 1.0;
            pathLine.id = 2;
            pathLine.type = visualization_msgs::Marker::LINE_STRIP;
            pathLine.scale.x = 0.15f;

        bestPath->visualize(&pathLine);
        // Heat map
        typename Trajectory<StateSpaceVector>::iterator itT;
        for (itT = bestPath->begin(); itT != bestPath->end(); itT++)
        {
            relativeVelocity = (*itT)->vx() / 10;
            if(relativeVelocity > 1.0f) relativeVelocity = 1.0f;
            varColor.r = relativeVelocity;
            varColor.g = 1 - relativeVelocity;
            pathLine.colors.push_back(varColor);
        }
        markerArray.markers.emplace_back(pathLine);
    }

    // Display status text
    visualization_msgs::Marker textInfo;
    textInfo.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textInfo.header.frame_id = "map";
    textInfo.header.stamp = ros::Time::now();
    textInfo.ns = "rrt_info";
    textInfo.action = visualization_msgs::Marker::ADD;
    textInfo.id = 3;
    textInfo.scale.x = 1.5,
    textInfo.scale.y = 1.5,
    textInfo.scale.z = 1.5;
    textInfo.color.r = 1,
    textInfo.color.g = 1,
    textInfo.color.b = 1,
    textInfo.color.a = 1;
    //textInfo.pose = {};
    textInfo.pose.position.x = tree->front()->getState()->x();
    textInfo.pose.position.y = tree->front()->getState()->y();
    textInfo.pose.position.z = 10;

    std::stringstream s;
    s << std::fixed << std::setprecision(1);

    s << "Node count: " << nodeCount << "\n";
    s << "Rewire count: " << rewireCount << "\n";
    s << "Path length: " << pathLength << " m\n";
    s << "Path time: " << pathTime << " s\n";

    textInfo.text = s.str();

    markerArray.markers.emplace_back(textInfo);
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::init(const shared_ptr<StateSpaceVector>& startState)
{
    tree->clear();
    tree->push_back(make_shared<SearchTreeNode<StateSpaceVector>> (SearchTreeNode<StateSpaceVector>(NULL, startState, 0)));
    vEdges.clear();
    // bestPath is not reinitialized as if local planning fails, it can reuse previous path

    // Initialize status variables
    nodeCount = 1;
    rewireCount = 0;
    pathFound = false;
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::init(shared_ptr<Trajectory<StateSpaceVector>> initPath)
{
    typename Trajectory<StateSpaceVector>::iterator it;
    int i = 0;
    float cost;
    Trajectory<StateSpaceVector> segment;
    segment.push_back(initPath->front());

    // Initialize tree
    init(initPath->front());

    // Add remaining states
    shared_ptr<SearchTreeNode<StateSpaceVector>> node = tree->front();
    for (it = initPath->begin() + 1; it != initPath->end(); it++)
    {
        if(!alreadyInTree(*it))
        {
            // Calculate cost
            segment.push_back(*it);
            cost = segment.getTimeCost();

            // Add state to tree
            node = addChild(node, *it, cost);

            // Reset segment
            segment.erase(segment.begin());
        }
    }
}

template<class StateSpaceVector>
shared_ptr<StateSpaceVector> SearchTree<StateSpaceVector>::getRoot() const
{
    return tree->front()->getState();
}

template<class StateSpaceVector>
shared_ptr<Trajectory<StateSpaceVector>> SearchTree<StateSpaceVector>::traceBackToRoot(const shared_ptr<StateSpaceVector>& goalState) const
{
    shared_ptr<SearchTreeNode<StateSpaceVector>> closestNode = getNearest(goalState, param->minCost);
    if (closestNode == NULL) return NULL;

    return traceBackToRoot(closestNode);
}

template<class StateSpaceVector>
shared_ptr<Trajectory<StateSpaceVector>> SearchTree<StateSpaceVector>::traceBackToRoot(const shared_ptr<SearchTreeNode<StateSpaceVector>>& node) const
{
    shared_ptr<Trajectory<StateSpaceVector>> path = shared_ptr<Trajectory<StateSpaceVector>> (new Trajectory<StateSpaceVector>());
    node->traceBackToRoot(path);
    return path;
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::updatePath(const shared_ptr<Trajectory<StateSpaceVector>>& path)
{
    bestPath = path;
    pathLength = bestPath->getDistanceCost();
    pathTime = bestPath->getTimeCost();
}

template<class StateSpaceVector>
bool SearchTree<StateSpaceVector>::addLoop(const shared_ptr<SearchTreeNode<StateSpaceVector>> startNode,
                                           const shared_ptr<SearchTreeNode<StateSpaceVector>> endNode, const float& cost)
{
    shared_ptr<Trajectory<StateSpaceVector>> path = traceBackToRoot(startNode);
    bool isLoop = false;

    typename Trajectory<StateSpaceVector>::iterator endIterator;

    endIterator = find(path->begin(), path->end(), endNode->getState());
    if (endIterator != path->end())
    {
        // A loop is detected
        vEdge<StateSpaceVector> virtEdge = {startNode, endNode, cost};
        vEdges.push_back(make_shared<vEdge<StateSpaceVector>>(virtEdge));
        isLoop = true;
    }

    return isLoop;
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::manageLoops(const unique_ptr<VEHICLE_PARAMETERS>& vParam)
{
    typename vector<shared_ptr<vEdge<StateSpaceVector>>>::iterator it;
    for(it = vEdges.begin(); it < vEdges.end(); it++)
    {
        bool isLoop = false;
        shared_ptr<vEdge<StateSpaceVector>> virtEdge = *it;
        shared_ptr<Trajectory<StateSpaceVector>> path = traceBackToRoot(virtEdge->start);
        if(path->size() > 1)
        {
            typename Trajectory<StateSpaceVector>::iterator endIterator;

            endIterator = find(path->begin(), path->end(), virtEdge->end->getState());
            if (endIterator != path->end())
            {
                // A loop is detected
                pathFound = true;
                isLoop = true;

                // Calculate cost of loop
                float cost = 0;
                cost = getAbsCost(virtEdge->start) - getAbsCost(virtEdge->end) + virtEdge->cost;

                // bestPath should be updated if cost is better or bestPath has not been created
                if(cost < pathCost || bestPath->size() == 0)
                {
                    // Remove states from path before end state
                    path->erase(path->begin(), endIterator);

                    // Add end state to the end to create a loop
                    path->push_back(virtEdge->end->getState());

                    // Update path related variables
                    pathCost = cost;
                    updatePath(path->getSimulated(param, vParam));
                }
            }
        }
        
        // If virtEdge does not create a loop anymore, delete it
        if(!isLoop)
        {
            it = vEdges.erase(std::remove(vEdges.begin(), vEdges.end(), virtEdge), vEdges.end());
        }
    }
}

template<class StateSpaceVector>
float SearchTree<StateSpaceVector>::getAbsCost(const shared_ptr<SearchTreeNode<StateSpaceVector>>& node) const
{
    float absCost = 0;
    node->addToAbsoluteCost(&absCost);
    return absCost;
}

template<class StateSpaceVector>
shared_ptr<Trajectory<StateSpaceVector>> SearchTree<StateSpaceVector>::getBestPath(void)
{
    return bestPath;
}

template<class StateSpaceVector>
bool SearchTree<StateSpaceVector>::maxNumOfNodesReached() const
{
    return (nodeCount == param->maxNumOfNodes);
}

template<class StateSpaceVector>
void SearchTree<StateSpaceVector>::rewire(shared_ptr<SearchTreeNode<StateSpaceVector>> node, shared_ptr<SearchTreeNode<StateSpaceVector>> newParent)
{
    if(node->getParent() != NULL) node->getParent()->removeChild(node);
    node->changeParent(newParent, node);
    rewireCount++;
}

template<class StateSpaceVector>
shared_ptr<SearchTreeNode<StateSpaceVector>> SearchTree<StateSpaceVector>::getRandomNode(void) const
{
    int ID = rand() % nodeCount;
    return (*tree)[ID];
}

// Define classes
template class SearchTree<StateSpace2D>;
template class SearchTree<KinematicBicycle>;
template class SearchTree<DynamicBicycle>;

template struct vEdge<StateSpace2D>;
template struct vEdge<KinematicBicycle>;
template struct vEdge<DynamicBicycle>;