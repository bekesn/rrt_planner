#ifndef SEARCHTREENODE_H
#define SEARCHTREENODE_H

#include <vector>
#include <iterator>
#include "Types.h"
#include "StateSpaceSimulated.h"
#include "Trajectory.h"
#include <cereal/cereal.hpp> // for defer
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>

class SearchTreeNode
{
private:
    shared_ptr<SearchTreeNode> parentNode;
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> childNodes;
    shared_ptr<SS_VECTOR> state;
    float cost;

public:

    // Constructor
    SearchTreeNode();
    SearchTreeNode(const SearchTreeNode &original);
    SearchTreeNode(shared_ptr<SearchTreeNode> parent, shared_ptr<SS_VECTOR> stateSpace, double nodeCost);

    // Destructor
    ~SearchTreeNode();

    // Add child node
    void addChild(shared_ptr<SearchTreeNode> childNode);

    // Remove child node
    void removeChild(shared_ptr<SearchTreeNode> childNode);

    // Get and change parent
    shared_ptr<SearchTreeNode> getParent() const;
    void changeParent(shared_ptr<SearchTreeNode> newParent, shared_ptr<SearchTreeNode>& selfPtr);

    // Get children
    shared_ptr<vector<shared_ptr<SearchTreeNode>>> getChildren() const;

    // Get state
    shared_ptr<SS_VECTOR> getState();

    // Cost
    float getSegmentCost(void) const;
    void changeSegmentCost(float newCostValue);
    void addToAbsoluteCost(float* absCost) const;

    // Trace back to parent and add state
    void traceBackToRoot(shared_ptr<PATH_TYPE>& stateVector) const;

    // Archive function for cereal
    template<class Archive>
    void serialize(Archive & archive){archive(cereal::defer(CEREAL_NVP(childNodes)),
                    cereal::defer(CEREAL_NVP(parentNode)), CEREAL_NVP(state), CEREAL_NVP(cost));}

};



#endif //SEARCHTREENODE_H