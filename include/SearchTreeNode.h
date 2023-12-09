#ifndef SEARCHTREENODE_H
#define SEARCHTREENODE_H

#include <vector>
#include <iterator>
#include "Types.h"
#include "DynamicBicycle.h"
#include "Trajectory.h"

template<class StateSpaceVector>
class SearchTreeNode
{
private:
    shared_ptr<SearchTreeNode> parentNode;
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> childNodes;
    shared_ptr<StateSpaceVector> state;
    float cost;

public:

    // Constructor
    SearchTreeNode();
    SearchTreeNode(const SearchTreeNode<StateSpaceVector> &original);
    SearchTreeNode(shared_ptr<SearchTreeNode<StateSpaceVector>> parent, shared_ptr<StateSpaceVector> stateSpace, double nodeCost);

    // Destructor
    ~SearchTreeNode();

    // Add child node
    void addChild(shared_ptr<SearchTreeNode<StateSpaceVector>> childNode);

    // Remove child node
    void removeChild(shared_ptr<SearchTreeNode<StateSpaceVector>> childNode);

    // Get and change parent
    shared_ptr<SearchTreeNode<StateSpaceVector>> getParent() const;
    void changeParent(shared_ptr<SearchTreeNode<StateSpaceVector>> newParent, shared_ptr<SearchTreeNode<StateSpaceVector>>& selfPtr);

    // Get children
    shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> getChildren() const;

    // Get state
    shared_ptr<StateSpaceVector> getState();

    // Cost
    float getSegmentCost(void) const;
    void changeSegmentCost(float newCostValue);
    void addToAbsoluteCost(float* absCost) const;

    // Trace back to parent and add state
    void traceBackToRoot(shared_ptr<Trajectory<StateSpaceVector>>& stateVector) const;
};



#endif //SEARCHTREENODE_H