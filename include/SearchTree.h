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
    SearchTree(const VehicleModel* vehicleModel, SS_VECTOR startState, const char* ID);

    //Destructor
    ~SearchTree();

    //Add child node
    SearchTreeNode* addChild(SearchTreeNode* parentNode, SS_VECTOR state, double nodeCost);

    //Remove node
    void remove(SearchTreeNode* node);

    //Get nearest node
    SearchTreeNode* getNearest(const SS_VECTOR* state, float minCost = 0.0f) const;

    //Get nearby nodes
    std::vector<SearchTreeNode*>* getNearby(SearchTreeNode* node) const;

    // Decide whether almost similar state already exists
    bool alreadyInTree(const SS_VECTOR* state) const;

    // Draw tree as lines
    void visualize(void);

    // Delete tree and create new
    void init(const SS_VECTOR* startState);
    void init(PATH_TYPE* initPath);

    // Get root
    SS_VECTOR* getRoot() const;

    // Traceback to root
    PATH_TYPE* traceBackToRoot(const SS_VECTOR* goalState) const;

    // Get absolute cost to node
    float getAbsCost(const SearchTreeNode* node) const;

    // Check wether number of nodes reached the maximum
    bool maxNumOfNodesReached() const;

    // Rewiring node from former parent to newParent node
    void rewire(SearchTreeNode* node, SearchTreeNode* newParent);

};



#endif //SEARCHTREE_H