#ifndef SEARCHTREENODE_H
#define SEARCHTREENODE_H

#include <vector>
#include <ros/ros.h>
#include <iterator>
#include "Types.h"
#include "StateSpaceSimulated.h"
#include "Trajectory.h"
#include <cereal/cereal.hpp> // for defer
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>

using namespace std;

class SearchTreeNode
{
private:
    SearchTreeNode* parentNode;
    vector<unique_ptr<SearchTreeNode>> *childNodes;
    SS_VECTOR state;
    float cost;
    bool isRoot;

public:

    // Constructor
    SearchTreeNode();
    SearchTreeNode(const SearchTreeNode &original);
    SearchTreeNode(SearchTreeNode* parent, SS_VECTOR stateSpace, double nodeCost);

    // Destructor
    ~SearchTreeNode();

    // Add child node
    void addChild(SearchTreeNode* childNode);

    // Remove child node
    void removeChild(SearchTreeNode* childNode);

    // Get and change parent
    SearchTreeNode* getParent() const;
    void changeParent(SearchTreeNode* newParent);

    // Get children
    vector<unique_ptr<SearchTreeNode>> *getChildren() const;

    // Get state
    SS_VECTOR* getState();

    // Cost
    float getSegmentCost(void) const;
    void changeSegmentCost(float newCostValue);
    void addToAbsoluteCost(float* absCost) const;

    // Trace back to parent and add state
    void traceBackToRoot(PATH_TYPE* stateVector) const;

    // Set isRoot property
    void setRoot(bool isNodeRoot);

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(*childNodes, state, cost, isRoot);}

};



#endif //SEARCHTREENODE_H