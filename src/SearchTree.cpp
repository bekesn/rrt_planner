#include <SearchTree.h>


SearchTree::SearchTree()
{
    param = new RRT_PARAMETERS;
    tree = new std::vector<SearchTreeNode*>;
    loopClosingNodes = new std::vector<SearchTreeNode*>;
    type = LOCAL_RRT;
    bestPath = new PATH_TYPE;
}

SearchTree::SearchTree(const VehicleModel* vehicleModel, SS_VECTOR startState, RRT_TYPE rrtType)
{
    param = new RRT_PARAMETERS;
    loopClosingNodes = new std::vector<SearchTreeNode*>(0);
    type = rrtType;
    bestPath = new PATH_TYPE;

    this->init(&startState);

    pathLength = 0;
    pathTime = 0;
}

SearchTree::~SearchTree()
{
    delete tree->front();
    delete tree;
    delete bestPath;
    delete loopClosingNodes;
    delete param;
}

SearchTreeNode* SearchTree::addChild(SearchTreeNode* parentNode, SS_VECTOR state, double nodeCost)
{
    if (!maxNumOfNodesReached())
    {
        tree->push_back(new SearchTreeNode(parentNode, state, nodeCost));
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

void SearchTree::remove(SearchTreeNode* node)
{

}

SearchTreeNode* SearchTree::getNearest(const SS_VECTOR* toState, float minCost) const
{
    std::vector<SearchTreeNode*>::iterator it;
    double minDist;
    double dist;
    SearchTreeNode* closest;

    // Initialize closest node and distance
    minDist = tree->front()->getState()->getDistToTarget(toState, param);
    closest = tree->front();

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if(getAbsCost(*it) >= minCost)
        {
            dist = (*it)->getState()->getDistToTarget(toState, param);
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

std::vector<SearchTreeNode*>* SearchTree::getNearby(SearchTreeNode* node) const
{
    std::vector<SearchTreeNode*>::iterator it;
    std::vector<SearchTreeNode*>* closeNodes = new std::vector<SearchTreeNode*>;

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if ((node->getState()->getDistToTarget((*it)->getState(), param) < (param->rewireRange)) && ((*it) != node))
        {
            closeNodes->push_back((*it));
        }
    }

    return closeNodes;

}

bool SearchTree::alreadyInTree(const SS_VECTOR* state) const
{
    SS_VECTOR* closest = getNearest(state)->getState();
    return abs(closest->getDistOriented(state, param)) < param->minDeviation;
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
    
    std::vector<SearchTreeNode*>::iterator it;
    for (it = tree->begin(); it != tree->end(); it++)
    {
        coord.x = (*it)->getState()->x();
        coord.y = (*it)->getState()->y();
        treeNodes.points.push_back(coord);

        relativeVelocity = (*it)->getState()->v() / param->maxVelocity;
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

    std::vector<SearchTreeNode*>::iterator treeIterator;
    vector<unique_ptr<SearchTreeNode>>::iterator childIterator;
    std::vector<unique_ptr<SearchTreeNode>> *children;

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
            
            coord.x = (*pathIterator).x();
            coord.y = (*pathIterator).y();
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

void SearchTree::init(const SS_VECTOR* startState)
{
    if (tree != NULL){
        delete tree->front();
        delete tree;
        delete loopClosingNodes;
    }
    tree = new std::vector<SearchTreeNode*>(0);
    tree->push_back(new SearchTreeNode(NULL, *startState, 0));
    loopClosingNodes = new std::vector<SearchTreeNode*>(0);

    // Initialize status variables
    nodeCount = 1;
    rewireCount = 0;
    pathClosed = false;
    pathFound = false;
}

void SearchTree::init(PATH_TYPE* initPath)
{
    PATH_TYPE::iterator it;
    int i = 0;
    float cost;
    PATH_TYPE segment;
    segment.push_back(initPath->front());

    // Initialize tree
    init(&initPath->front());

    // Add remaining states
    SearchTreeNode* node = tree->front();
    for (it = initPath->begin() + 1; it != initPath->end(); it++)
    {
        if(!alreadyInTree(&(*it)))
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

SS_VECTOR* SearchTree::getRoot() const
{
    return tree->front()->getState();
}

PATH_TYPE* SearchTree::traceBackToRoot(const SS_VECTOR* goalState) const
{
    SearchTreeNode* closestNode = getNearest(goalState, param->minCost);
    if (closestNode == NULL) return NULL;

    PATH_TYPE* path = new PATH_TYPE;
    closestNode->traceBackToRoot(path);
    return path;
}

float SearchTree::getAbsCost(const SearchTreeNode* node) const
{
    float absCost = 0;
    node->addToAbsoluteCost(&absCost);
    return absCost;
}

bool SearchTree::maxNumOfNodesReached() const
{
    return (nodeCount == param->maxNumOfNodes);
}

void SearchTree::rewire(SearchTreeNode* node, SearchTreeNode* newParent)
{
    if(node->getParent() != NULL) node->getParent()->removeChild(node);
    node->changeParent(newParent);
    rewireCount++;
}
