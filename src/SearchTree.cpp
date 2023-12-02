#include <SearchTree.h>


SearchTree::SearchTree()
{
    param = unique_ptr<RRT_PARAMETERS> (new RRT_PARAMETERS);
    type = LOCAL_RRT;
    bestPath = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    tree = unique_ptr<vector<shared_ptr<SearchTreeNode>>> (new vector<shared_ptr<SearchTreeNode>>);

    pathLength = 0;
    pathTime = 0;
    pathCost = 0;
}

SearchTree::SearchTree(shared_ptr<SS_VECTOR> startState, RRT_TYPE rrtType) : SearchTree()
{
    type = rrtType;

    this->init(startState);
}

SearchTree::~SearchTree()
{

}

shared_ptr<SearchTreeNode> SearchTree::addChild(shared_ptr<SearchTreeNode> parentNode, shared_ptr<SS_VECTOR> state, double nodeCost)
{
    if (!maxNumOfNodesReached())
    {
        tree->push_back(make_shared<SearchTreeNode>(SearchTreeNode(parentNode, state, nodeCost)));
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

void SearchTree::remove(shared_ptr<SearchTreeNode> node)
{
    int numOfChildren = node->getChildren()->size();
    if(numOfChildren == 0)
    {
        node->getParent()->removeChild(node);
        tree->erase(std::remove(tree->begin(), tree->end(), node),tree->end());
        nodeCount--;
    }
    else
    {
        ROS_WARN_STREAM("Trying to delete node with " << numOfChildren << " children.");
    }
}

shared_ptr<SearchTreeNode> SearchTree::getNearest(const shared_ptr<SS_VECTOR>& toState, float minCost) const
{
    std::vector<shared_ptr<SearchTreeNode>>::iterator it;
    double minDist;
    double dist;
    shared_ptr<SearchTreeNode> closest;

    // Initialize closest node and distance
    minDist = tree->front()->getState()->getDistToTarget(*toState, param);
    closest = tree->front();

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if(getAbsCost(*it) >= minCost)
        {
            dist = (*it)->getState()->getDistToTarget(*toState, param);
            if (dist < minDist)
            {
                minDist = dist;
                closest = (*it);
            }
        }
    }

    if(getAbsCost(closest) < minCost) closest = NULL;

    return closest;
}

shared_ptr<std::vector<shared_ptr<SearchTreeNode>>> SearchTree::getNearby(shared_ptr<SearchTreeNode> node) const
{
    vector<shared_ptr<SearchTreeNode>>::iterator it;
    auto closeNodes = shared_ptr<vector<shared_ptr<SearchTreeNode>>> (new vector<shared_ptr<SearchTreeNode>>);
    shared_ptr<SS_VECTOR> state = node->getState();

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if ((state->getDistToTarget(*(*it)->getState(), param) < (param->rewireTime * param->simulationTimeStep * state->v())) && ((*it) != node))
        {
            closeNodes->push_back((*it));
        }
    }

    return closeNodes;

}

bool SearchTree::alreadyInTree(const shared_ptr<SS_VECTOR>& state) const
{
    std::vector<shared_ptr<SearchTreeNode>>::iterator it;
    bool isInTree = false;

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if(state->getDistEuclidean(*(*it)->getState()) < param->minDeviationDist)
        {
           isInTree = true;
           break;
        }
        else if(state->getDistOriented(*(*it)->getState(), param) < param->minDeviation)
        {
           isInTree = true;
           break;
        }
    }

    return isInTree;
}

void SearchTree::visualize(void)
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
        /*treeNodes.color.r = 0.176f;
        treeNodes.color.g = 0.658f;
        treeNodes.color.b = 0.105f;
        treeNodes.color.a = 1.0f;*/

    geometry_msgs::Point coord;
    
    vector<shared_ptr<SearchTreeNode>>::iterator it;
    for (it = tree->begin(); it != tree->end(); it++)
    {
        coord.x = (*it)->getState()->x();
        coord.y = (*it)->getState()->y();
        treeNodes.points.push_back(coord);

        relativeVelocity = (*it)->getState()->v() / 10;
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

    vector<shared_ptr<SearchTreeNode>>::iterator treeIterator;
    vector<shared_ptr<SearchTreeNode>>::iterator childIterator;
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> children;

    for (treeIterator = tree->begin(); treeIterator != tree->end(); treeIterator++)
    {
        children = (*treeIterator)->getChildren();
        for (childIterator = children->begin(); childIterator != children->end(); childIterator++)
        {
            coord.x = (*treeIterator)->getState()->x();
            coord.y = (*treeIterator)->getState()->y();
            graphEdge.points.push_back(coord);
            coord.x = (*childIterator)->getState()->x();
            coord.y = (*childIterator)->getState()->y();
            graphEdge.points.push_back(coord);
        }
    }


    markerArray.markers.emplace_back(graphEdge);
    

    if (this->pathFound && (bestPath->size() > 1))
    {        
        // Best path visualization
        visualization_msgs::Marker bestPathLine;
            bestPathLine.header.frame_id = "map";
            bestPathLine.header.stamp = ros::Time::now();
            bestPathLine.ns = "rrt_best_path";
            bestPathLine.action = visualization_msgs::Marker::ADD;
            bestPathLine.pose.orientation.w = 1.0;
            bestPathLine.id = 2;
            bestPathLine.type = visualization_msgs::Marker::LINE_STRIP;
            bestPathLine.scale.x = 0.15f;
            bestPathLine.color.r = 0.0f;
            bestPathLine.color.g = 0.5f;
            bestPathLine.color.b = 1.0f;
            bestPathLine.color.a = 1.0f;

        PATH_TYPE::iterator pathIterator;
        for (pathIterator = this->bestPath->begin(); pathIterator != this->bestPath->end(); pathIterator++)
        {
            
            coord.x = (*pathIterator)->x();
            coord.y = (*pathIterator)->y();
            bestPathLine.points.push_back(coord);

        }
        
        markerArray.markers.emplace_back(bestPathLine);
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

void SearchTree::init(const shared_ptr<SS_VECTOR>& startState)
{
    tree->clear();
    tree->push_back(make_shared<SearchTreeNode> (SearchTreeNode(NULL, startState, 0)));

    // Initialize status variables
    nodeCount = 1;
    rewireCount = 0;
    pathFound = false;
}

void SearchTree::init(shared_ptr<PATH_TYPE> initPath)
{
    PATH_TYPE::iterator it;
    int i = 0;
    float cost;
    PATH_TYPE segment;
    segment.push_back(initPath->front());

    // Initialize tree
    init(initPath->front());

    // Add remaining states
    shared_ptr<SearchTreeNode> node = tree->front();
    for (it = initPath->begin() + 1; it != initPath->end(); it++)
    {
        if(!alreadyInTree(*it))
        {
            // Calculate cost
            segment.push_back(*it);
            cost = segment.cost(param);

            // Add state to tree
            node = addChild(node, *it, cost);

            // Reset segment
            segment.erase(segment.begin());
        }
    }
}

shared_ptr<SS_VECTOR> SearchTree::getRoot() const
{
    return tree->front()->getState();
}

shared_ptr<PATH_TYPE> SearchTree::traceBackToRoot(const shared_ptr<SS_VECTOR>& goalState) const
{
    shared_ptr<SearchTreeNode> closestNode = getNearest(goalState, param->minCost);
    if (closestNode == NULL) return NULL;

    return traceBackToRoot(closestNode);
}

shared_ptr<PATH_TYPE> SearchTree::traceBackToRoot(const shared_ptr<SearchTreeNode>& node) const
{
    shared_ptr<PATH_TYPE> path = shared_ptr<PATH_TYPE> (new PATH_TYPE);
    node->traceBackToRoot(path);
    return path;
}

void SearchTree::updatePath(shared_ptr<SearchTreeNode>& endNode)
{
    bestPath = traceBackToRoot(endNode);
    pathLength = bestPath->getDistanceCost();
    pathTime = bestPath->getTimeCost();
}

bool SearchTree::closeLoop(const shared_ptr<SearchTreeNode>& startNode, const shared_ptr<SearchTreeNode>& endNode)
{
    shared_ptr<PATH_TYPE> path = traceBackToRoot(endNode);
    bool isLoop = false;
    float cost = 0;

    PATH_TYPE::iterator startIterator;

    startIterator = find(path->begin(), path->end(), startNode->getState());
    if (startIterator != path->end())
    {
        // Remove states from path before start state
        path->erase(path->begin(), startIterator);
        
        // Add start state to the end to create a loop
        path->push_back(startNode->getState());
        
        isLoop = true;
        cost = path->cost(param);
    }

    // If a loop is found:
    // If no loop was defined before, this will be the loop
    // If loop was defined but this has a lower cost, loop is replaced
    if (isLoop && (!pathFound || (cost < pathCost)))
    {
        pathFound = true;

        // Update path and its costs
        bestPath = path;
        pathCost = cost;
        pathLength = bestPath->getDistanceCost();
        pathTime = bestPath->getTimeCost();
    }

    return isLoop;
}

float SearchTree::getAbsCost(const shared_ptr<SearchTreeNode>& node) const
{
    float absCost = 0;
    node->addToAbsoluteCost(&absCost);
    return absCost;
}

shared_ptr<PATH_TYPE> SearchTree::getBestPath(void)
{
    return bestPath;
}

bool SearchTree::maxNumOfNodesReached() const
{
    return (nodeCount == param->maxNumOfNodes);
}

void SearchTree::rewire(shared_ptr<SearchTreeNode> node, shared_ptr<SearchTreeNode> newParent)
{
    if(node->getParent() != NULL) node->getParent()->removeChild(node);
    node->changeParent(newParent, node);
    rewireCount++;
}

shared_ptr<SearchTreeNode> SearchTree::getRandomNode(void) const
{
    int ID = rand() % nodeCount;
    return (*tree)[ID];
}
