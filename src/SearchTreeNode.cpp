#include <SearchTreeNode.h>

template<class StateSpaceVector>
SearchTreeNode<StateSpaceVector>::SearchTreeNode()
{
    parentNode = shared_ptr<SearchTreeNode<StateSpaceVector>> (new SearchTreeNode<StateSpaceVector>);
    cost = 0;
    childNodes = shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> (new vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>);
}

template<class StateSpaceVector>
SearchTreeNode<StateSpaceVector>::SearchTreeNode(const SearchTreeNode<StateSpaceVector> &original)
{
    parentNode = original.parentNode;
    childNodes = original.childNodes;
    state = original.state;
    cost = original.cost;
}

template<class StateSpaceVector>
SearchTreeNode<StateSpaceVector>::SearchTreeNode(shared_ptr<SearchTreeNode<StateSpaceVector>> parent, shared_ptr<StateSpaceVector>stateSpace, double nodeCost)
{
    parentNode = parent;
    state = stateSpace;
    cost = nodeCost;
    childNodes = shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> (new vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>);
}

template<class StateSpaceVector>
SearchTreeNode<StateSpaceVector>::~SearchTreeNode()
{

}

template<class StateSpaceVector>
void SearchTreeNode<StateSpaceVector>::addChild(shared_ptr<SearchTreeNode<StateSpaceVector>> childNode)
{
    childNodes->push_back(childNode);
}

template<class StateSpaceVector>
void SearchTreeNode<StateSpaceVector>::removeChild(shared_ptr<SearchTreeNode<StateSpaceVector>> childNode)
{
    childNodes->erase(std::remove(childNodes->begin(), childNodes->end(), childNode),childNodes->end());
}

template<class StateSpaceVector>
shared_ptr<SearchTreeNode<StateSpaceVector>> SearchTreeNode<StateSpaceVector>::getParent() const
{
    return parentNode;
}

template<class StateSpaceVector>
void SearchTreeNode<StateSpaceVector>::changeParent(shared_ptr<SearchTreeNode<StateSpaceVector>> newParent, shared_ptr<SearchTreeNode<StateSpaceVector>>& selfPtr)
{
    parentNode = newParent;
    newParent->addChild(selfPtr);
}

template<class StateSpaceVector>
shared_ptr<vector<shared_ptr<SearchTreeNode<StateSpaceVector>>>> SearchTreeNode<StateSpaceVector>::getChildren() const
{
    return childNodes;
}

template<class StateSpaceVector>
shared_ptr<StateSpaceVector> SearchTreeNode<StateSpaceVector>::getState()
{
    return state;
}

template<class StateSpaceVector>
float SearchTreeNode<StateSpaceVector>::getSegmentCost(void) const
{
    return cost;
}

template<class StateSpaceVector>
void SearchTreeNode<StateSpaceVector>::changeSegmentCost(float newCostValue)
{
    cost = newCostValue;
}

template<class StateSpaceVector>
void SearchTreeNode<StateSpaceVector>::addToAbsoluteCost(float* absCost) const
{
    if (parentNode != NULL) 
    {
        *absCost += cost;
        parentNode->addToAbsoluteCost(absCost);
    }
}

template<class StateSpaceVector>
void SearchTreeNode<StateSpaceVector>::traceBackToRoot(shared_ptr<Trajectory<StateSpaceVector>>& stateVector) const
{
    // TODO: inserting at the beginning might slow down the process
    stateVector->insert(stateVector->begin(), state);
    if (parentNode != NULL) parentNode->traceBackToRoot(stateVector);
}

// Define classes
template class SearchTreeNode<StateSpace2D>;
template class SearchTreeNode<KinematicBicycle>;
template class SearchTreeNode<DynamicBicycle>;