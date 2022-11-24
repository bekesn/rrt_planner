#ifndef SEARCHTREE_H
#define SEARCHTREE_H

#include <vector>
#include <SearchTreeNode.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <VehicleModel.h>

class SearchTree
{
private:
    std::vector<SearchTreeNode*> *tree;
    VehicleModel* vehicle;
    int maxNumOfNodes = 1000;

public:

    //Constructor
    SearchTree();
    SearchTree(VehicleModel* vehicleModel, std::vector<double> startState);

    //Destructor
    ~SearchTree();

    //Add child node
    SearchTreeNode* addChild(SearchTreeNode* parentNode, std::vector<double> state, double nodeCost);

    //Remove node
    void remove(SearchTreeNode* node);

    //Get nearest node
    SearchTreeNode* getNearest(std::vector<double> state);

    //Get nearby nodes
    std::vector<SearchTreeNode*>* getNearby(SearchTreeNode* node, double maxDist);

    // Draw tree as lines
    void drawTree(visualization_msgs::MarkerArray* markerArray);

    // Delete tree and create new
    void reset(std::vector<double> startState);

    // Traceback to root
    std::vector<std::vector<double>>* traceBackToRoot(std::vector<double> goalState);

    // Get absolute cost to node
    float getAbsCost(SearchTreeNode* node);

    // Check wether number of nodes reached the maximum
    bool maxNumOfNodesReached();

    // Rewiring node from former parent to newParent node
    void rewire(SearchTreeNode* node, SearchTreeNode* newParent);

};



#endif //SEARCHTREE_H