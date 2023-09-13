#include <SearchTree.h>


SearchTree::SearchTree()
{
    param = new RRT_PARAMETERS;
    tree = new std::vector<SearchTreeNode*>;
    loopClosingNodes = new std::vector<SearchTreeNode*>;
    name = new std::string;
    bestPath = new PATH_TYPE(0);
}

SearchTree::SearchTree(VehicleModel* vehicleModel, SS_VECTOR startState, const char* ID)
{
    param = new RRT_PARAMETERS;
    loopClosingNodes = new std::vector<SearchTreeNode*>(0);
    name = new std::string(1, *ID);
    bestPath = new PATH_TYPE(0);

    this->init(&startState);

    pathCost = 0;
}

SearchTree::~SearchTree()
{
    delete tree->front();
    delete tree;
    delete bestPath;
    delete loopClosingNodes;
    delete name;
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

SearchTreeNode* SearchTree::getNearest(SS_VECTOR* toState, float minCost)
{
    std::vector<SearchTreeNode*>::iterator it;
    double minDist;
    double dist;
    SearchTreeNode* closest;

    // Initialize closest node and distance
    minDist = tree->front()->getState()->distanceToTarget(toState, param);
    closest = tree->front();

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if(getAbsCost(*it) >= minCost)
        {
            dist = (*it)->getState()->distanceToTarget(toState, param);
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

std::vector<SearchTreeNode*>* SearchTree::getNearby(SearchTreeNode* node)
{
    std::vector<SearchTreeNode*>::iterator it;
    std::vector<SearchTreeNode*>* closeNodes = new std::vector<SearchTreeNode*>;

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if (((*it)->getState()->distanceToTarget(node->getState(), param) < (param->rewireRange)) && ((*it) != node))
        {
            closeNodes->push_back((*it));
        }
    }

    return closeNodes;

}

bool SearchTree::alreadyInTree(SS_VECTOR* state)
{
    SS_VECTOR* closest = getNearest(state)->getState();
    return (closest->distanceToTarget(state, param) < param->minDeviation) || (state->distanceToTarget(closest, param) < param->minDeviation);
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
        graphEdge.scale.x = 0.1f;
        graphEdge.color.r = 1.0f;
        graphEdge.color.g = 0.65f;
        graphEdge.color.b = 0.0f;
        graphEdge.color.a = 1.0f;

    std::vector<SearchTreeNode*>::iterator treeIterator;
    std::vector<SearchTreeNode*>::iterator childIterator;
    std::vector<SearchTreeNode*> *children;

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
            bestPathLine.scale.x = 0.3f;
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
    s << "Path cost: " << pathCost << "\n";

    textInfo.text = s.str();

    markerArray.markers.emplace_back(textInfo);
}

void SearchTree::init(SS_VECTOR* startState)
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

SS_VECTOR* SearchTree::getRoot()
{
    return tree->front()->getState();
}

PATH_TYPE* SearchTree::traceBackToRoot(SS_VECTOR* goalState)
{
    SearchTreeNode* closestNode = getNearest(goalState, param->minCost);
    PATH_TYPE* path = new PATH_TYPE;
    closestNode->traceBackToRoot(path);
    return path;
}

float SearchTree::getAbsCost(SearchTreeNode* node)
{
    float absCost = 0;
    node->addToAbsoluteCost(&absCost);
    return absCost;
}

bool SearchTree::maxNumOfNodesReached()
{
    return (nodeCount == param->maxNumOfNodes);
}

void SearchTree::rewire(SearchTreeNode* node, SearchTreeNode* newParent)
{
    node->getParent()->removeChild(node);
    node->changeParent(newParent);
    rewireCount++;
}
