#include <SearchTree.h>


SearchTree::SearchTree()
{
    tree = new std::vector<SearchTreeNode*>;
    loopClosingNodes = new std::vector<SearchTreeNode*>;
}

SearchTree::SearchTree(VehicleModel* vehicleModel, SS_VECTOR startState, RRT_PARAMETERS* par)
{
    param = par;
    tree = new std::vector<SearchTreeNode*>(0);
    loopClosingNodes = new std::vector<SearchTreeNode*>(0);
    tree->push_back(new SearchTreeNode(NULL, startState, 0));
}

SearchTree::~SearchTree()
{
    delete tree->front();
    delete tree;
}

SearchTreeNode* SearchTree::addChild(SearchTreeNode* parentNode, SS_VECTOR state, double nodeCost)
{
    if (!maxNumOfNodesReached())
    {
        tree->push_back(new SearchTreeNode(parentNode, state, nodeCost));
        parentNode->addChild(tree->back());
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
    return getNearest(state)->getState()->distanceToTarget(state, param) < param->minDeviation;
}

void SearchTree::drawTree(visualization_msgs::MarkerArray* markerArray)
{
    if(tree->size() < 2) return;

    visualization_msgs::Marker treeNodes;
        treeNodes.header.frame_id = "map";
        treeNodes.header.stamp = ros::Time::now();
        treeNodes.ns = "rrt_states";
        treeNodes.action = visualization_msgs::Marker::ADD;
        treeNodes.pose.orientation.w = 1.0;
        treeNodes.id = 0;
        treeNodes.type = visualization_msgs::Marker::CUBE_LIST;
        treeNodes.scale.x = 0.3f;
        treeNodes.scale.y = 0.3f;
        treeNodes.scale.z = 0.3f;
        treeNodes.color.r = 0.176f;
        treeNodes.color.g = 0.658f;
        treeNodes.color.b = 0.105f;
        treeNodes.color.a = 1.0f;

    geometry_msgs::Point coord;
    
    std::vector<SearchTreeNode*>::iterator it;
    for (it = tree->begin(); it != tree->end(); it++)
    {
        coord.x = (*it)->getState()->x();
        coord.y = (*it)->getState()->y();
        treeNodes.points.push_back(coord);
    }

    markerArray->markers.emplace_back(treeNodes);

    // Draw lines
    visualization_msgs::Marker graphEdge;
        graphEdge.header.frame_id = "map";
        graphEdge.header.stamp = ros::Time::now();
        graphEdge.ns = "rrt_graph_edge";
        graphEdge.action = visualization_msgs::Marker::ADD;
        graphEdge.pose.orientation.w = 1.0;
        graphEdge.id = 1;
        graphEdge.type = visualization_msgs::Marker::LINE_LIST;
        graphEdge.scale.x = 0.3f;
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


    markerArray->markers.emplace_back(graphEdge);
    /*for (treeIterator = tree->begin(); treeIterator != tree->end(); treeIterator++)
    {
        ROS_INFO_STREAM("" << (*treeIterator)->getState()[0] << "  " << (*treeIterator)->getState()[1] << "    " << (*treeIterator)->getChildren()->size());
    }*/
}

void SearchTree::init(SS_VECTOR* startState)
{
    delete tree->front();
    delete tree;
    tree = new std::vector<SearchTreeNode*>;
    tree->push_back(new SearchTreeNode(NULL, *startState, 0));
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
    return (tree->size() == param->maxNumOfNodes);
}

void SearchTree::rewire(SearchTreeNode* node, SearchTreeNode* newParent)
{
    node->getParent()->removeChild(node);
    node->changeParent(newParent);
}
