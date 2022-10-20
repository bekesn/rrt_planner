#include <SearchTree.h>


SearchTree::SearchTree()
{
    tree = new std::vector<SearchTreeNode*>(0);
}

SearchTree::SearchTree(VehicleModel* vehicleModel, std::vector<double> startState)
{
    vehicle = vehicleModel;
    tree = new std::vector<SearchTreeNode*>(0);
    tree->push_back(new SearchTreeNode(NULL, startState));
}

void SearchTree::addChild(SearchTreeNode* parentNode, std::vector<double> state)
{
    if (tree->size() < maxNumOfNodes)
    {
        if (parentNode != NULL)
        {
            tree->push_back(new SearchTreeNode(parentNode, state));
            parentNode->addChild(tree->back());
        }
        else
        {
            ROS_ERROR("[RRT_PLANNER]: parentNode is NULL in SearchTree::addChild()");
        }
    }
}

void SearchTree::remove(SearchTreeNode* node)
{

}

SearchTreeNode* SearchTree::getNearest(std::vector<double> toState)
{
    //ROS_INFO_STREAM("new:  " << toState[0] << "  " << toState[1]);
    std::vector<SearchTreeNode*>::iterator it;
    double minDist;
    double dist;
    SearchTreeNode* closest;

    // Initialize closest node and distance
    minDist = vehicle->distance(tree->front()->getState(), toState);
    closest = tree->front();

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        dist = vehicle->distance((*it)->getState(), toState);
        if (dist < minDist)
        {
            minDist = dist;
            closest = (*it);
        }
    }

    //ROS_INFO_STREAM("closest:   " << closest->getState()[0] << "   " << closest->getState()[1]);

    return closest;
}

std::vector<SearchTreeNode*> SearchTree::getNearby(std::vector<double> toState, double maxDist)
{
    std::vector<SearchTreeNode*>::iterator it;
    std::vector<SearchTreeNode*> closeNodes;

    // Iterate through tree
    for (it = tree->begin(); it != tree->end(); it++)
    {
        if ((vehicle->distance((*it)->getState(), toState)) < maxDist)
        {
            closeNodes.push_back((*it));
        }
    }

    return closeNodes;

}

void SearchTree::drawTree(visualization_msgs::MarkerArray* markerArray)
{
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
        coord.x = (*it)->getState()[0];
        coord.y = (*it)->getState()[1];
        treeNodes.points.push_back(coord);
    }

    markerArray->markers.emplace_back(treeNodes);

    // Draw lines
    visualization_msgs::Marker graph_edge;
        graph_edge.header.frame_id = "map";
        graph_edge.header.stamp = ros::Time::now();
        graph_edge.ns = "rrt_graph_edge";
        graph_edge.action = visualization_msgs::Marker::ADD;
        graph_edge.pose.orientation.w = 1.0;
        graph_edge.id = 0;
        graph_edge.type = visualization_msgs::Marker::LINE_LIST;
        graph_edge.scale.x = 0.3f;
        graph_edge.color.r = 1.0f;
        graph_edge.color.g = 0.65f;
        graph_edge.color.b = 0.0f;
        graph_edge.color.a = 1.0f;

    std::vector<SearchTreeNode*>::iterator treeIterator;
    std::vector<SearchTreeNode*>::iterator childIterator;
    std::vector<SearchTreeNode*> *children;

    for (treeIterator = tree->begin(); treeIterator != tree->end(); treeIterator++)
    {
        children = (*treeIterator)->getChildren();
        for (childIterator = children->begin(); childIterator != children->end(); childIterator++)
        {
            coord.x = (*treeIterator)->getState()[0];
            coord.y = (*treeIterator)->getState()[1];
            graph_edge.points.push_back(coord);
            coord.x = (*childIterator)->getState()[0];
            coord.y = (*childIterator)->getState()[1];
            graph_edge.points.push_back(coord);
        }
    }


    markerArray->markers.emplace_back(graph_edge);
    /*for (treeIterator = tree->begin(); treeIterator != tree->end(); treeIterator++)
    {
        ROS_INFO_STREAM("" << (*treeIterator)->getState()[0] << "  " << (*treeIterator)->getState()[1] << "    " << (*treeIterator)->getChildren()->size());
    }*/
}
