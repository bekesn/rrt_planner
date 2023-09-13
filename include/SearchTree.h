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
    std::vector<SearchTreeNode*> *loopClosingNodes;

    std::string* name;

public:

    // Parameter struct
    RRT_PARAMETERS* param;

    int nodeCount;
    int rewireCount;
    float pathLength;
    float pathTime;
    bool pathClosed;
    bool pathFound;
    PATH_TYPE* bestPath;

    ros::Publisher markerPublisher;
    visualization_msgs::MarkerArray markerArray;

public:

    //Constructor
    SearchTree();
    SearchTree(VehicleModel* vehicleModel, SS_VECTOR startState, const char* ID);

    //Destructor
    ~SearchTree();

    //Add child node
    SearchTreeNode* addChild(SearchTreeNode* parentNode, SS_VECTOR state, double nodeCost);

    //Remove node
    void remove(SearchTreeNode* node);

    //Get nearest node
    SearchTreeNode* getNearest(SS_VECTOR* state, float minCost = 0.0f);

    //Get nearby nodes
    std::vector<SearchTreeNode*>* getNearby(SearchTreeNode* node);

    // Decide whether almost similar state already exists
    bool alreadyInTree(SS_VECTOR* state);

    // Draw tree as lines
    void visualize(void);

    // Delete tree and create new
    void init(SS_VECTOR* startState);

    // Get root
    SS_VECTOR* getRoot();

    // Traceback to root
    PATH_TYPE* traceBackToRoot(SS_VECTOR* goalState);

    // Get absolute cost to node
    float getAbsCost(SearchTreeNode* node);

    // Check wether number of nodes reached the maximum
    bool maxNumOfNodesReached();

    // Rewiring node from former parent to newParent node
    void rewire(SearchTreeNode* node, SearchTreeNode* newParent);

};



#endif //SEARCHTREE_H