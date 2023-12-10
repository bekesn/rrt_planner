#ifndef SEARCHTREE_H
#define SEARCHTREE_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "SearchTreeNode.h"

template<class StateSpaceVector>
struct vEdge;

template<class StateSpaceVector>
class SearchTree
{
private:
    shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> tree;
    vector<shared_ptr<vEdge<StateSpaceVector>>> vEdges;
    shared_ptr<Trajectory<StateSpaceVector>> bestPath;

    RRT_TYPE type;

    float pathCost;
    int nodeCount;
    int rewireCount;
    float pathLength;
    float pathTime;

public:
    bool pathFound;

    // Parameter struct
    unique_ptr<RRT_PARAMETERS> param;


    ros::Publisher markerPublisher;
    visualization_msgs::MarkerArray markerArray;

public:

    //Constructor
    SearchTree();
    SearchTree(shared_ptr<StateSpaceVector> startState, RRT_TYPE rrtType);

    //Destructor
    ~SearchTree();

    // Add child node
    // Return pointer to newly inserted node
    shared_ptr<SearchTreeNode<StateSpaceVector>> addChild(shared_ptr<SearchTreeNode<StateSpaceVector>> parentNode, shared_ptr<StateSpaceVector> state, double nodeCost);

    //Remove node
    void remove(shared_ptr<SearchTreeNode<StateSpaceVector>> node);

    //Get nearest node
    shared_ptr<SearchTreeNode<StateSpaceVector>> getNearest(const shared_ptr<StateSpaceVector>& state, float minCost = 0.0f) const;

    //Get nearby nodes
    shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> getNearby(shared_ptr<SearchTreeNode<StateSpaceVector>> node) const;

    // Decide whether almost similar state already exists
    bool alreadyInTree(const shared_ptr<StateSpaceVector>& state) const;

    // Draw tree as lines
    void visualize(unique_ptr<VEHICLE_PARAMETERS>& vParam);

    // Delete tree and create new
    void init(const shared_ptr<StateSpaceVector>& startState);
    void init(shared_ptr<Trajectory<StateSpaceVector>> initPath);

    // Get root
    shared_ptr<StateSpaceVector> getRoot() const;

    // Traceback to root from given state
    // Gets closest node to given state and returns with trajectory from root to node
    shared_ptr<Trajectory<StateSpaceVector>> traceBackToRoot(const shared_ptr<StateSpaceVector>& goalState) const;

    // Traceback to root from given node
    // Returns with trajectory from root to node
    shared_ptr<Trajectory<StateSpaceVector>> traceBackToRoot(const shared_ptr<SearchTreeNode<StateSpaceVector>>& node) const;

    // Update best path to given path
    void updatePath(const shared_ptr<Trajectory<StateSpaceVector>>& path);

    // Create closing segment to define best loop
    // Investigates if a real loop is defined
    // Investigates if cost is decreased compared to the previous cost
    // Return whether the best loop was changed
    bool addLoop(const shared_ptr<SearchTreeNode<StateSpaceVector>> startNode, const shared_ptr<SearchTreeNode<StateSpaceVector>> endNode, const float& cost);
    void manageLoops(const unique_ptr<VEHICLE_PARAMETERS>& vParam);

    // Get absolute cost to node
    float getAbsCost(const shared_ptr<SearchTreeNode<StateSpaceVector>>& node) const;

    // Get best path
    shared_ptr<Trajectory<StateSpaceVector>> getBestPath(void);

    // Check wether number of nodes reached the maximum
    bool maxNumOfNodesReached() const;

    // Rewiring node from former parent to newParent node
    void rewire(shared_ptr<SearchTreeNode<StateSpaceVector>> node, shared_ptr<SearchTreeNode<StateSpaceVector>> newParent);

    // Provide a randomly chosen node
    shared_ptr<SearchTreeNode<StateSpaceVector>> getRandomNode(void) const;
};

template<class StateSpaceVector>
struct vEdge{
    shared_ptr<SearchTreeNode<StateSpaceVector>> start;
    shared_ptr<SearchTreeNode<StateSpaceVector>> end;
    float cost;
};


#endif //SEARCHTREE_H