#include <SearchTreeNode.h>

SearchTreeNode::SearchTreeNode()
{
    childNodes = new vector<unique_ptr<SearchTreeNode>>;
    isRoot = false;
}


SearchTreeNode::SearchTreeNode(const SearchTreeNode &original)
{
    parentNode = original.parentNode;
    //childNodes = original.childNodes;
    childNodes = new vector<unique_ptr<SearchTreeNode>>;
    move(original.childNodes->begin(), original.childNodes->end(), back_inserter(*childNodes));
    state = original.state;
    cost = original.cost;
    isRoot = original.isRoot;
}

SearchTreeNode::SearchTreeNode(SearchTreeNode* parent, SS_VECTOR stateSpace, double nodeCost)
{
    parentNode = parent;
    state = stateSpace;
    childNodes = new vector<unique_ptr<SearchTreeNode>>;
    cost = nodeCost;
    isRoot = false;
}

SearchTreeNode::~SearchTreeNode()
{
    vector<unique_ptr<SearchTreeNode>>::iterator it;
    /*for (it = childNodes->begin(); it != childNodes->end(); it++)
    {
        delete (*it).get();
    }*/
    //delete childNodes;
}


void SearchTreeNode::addChild(SearchTreeNode* childNode)
{
    childNodes->push_back(unique_ptr<SearchTreeNode>(childNode));
}

void SearchTreeNode::removeChild(SearchTreeNode* childNode)
{
    //childNodes->erase(std::remove(childNodes->begin(), childNodes->end(), childNode),childNodes->end());
    childNodes->erase(remove_if(childNodes->begin(), childNodes->end(),
                        [childNode](unique_ptr<SearchTreeNode>& unique){return unique.get() == childNode;}));
}

SearchTreeNode* SearchTreeNode::getParent() const
{
    return parentNode;
}

void SearchTreeNode::changeParent(SearchTreeNode* newParent)
{
    parentNode = newParent;
    newParent->addChild(this);
}

vector<unique_ptr<SearchTreeNode>>* SearchTreeNode::getChildren() const
{
    return childNodes;
}

SS_VECTOR* SearchTreeNode::getState()
{
    return &state;
}

float SearchTreeNode::getSegmentCost(void) const
{
    return cost;
}

void SearchTreeNode::changeSegmentCost(float newCostValue)
{
    cost = newCostValue;
}

void SearchTreeNode::addToAbsoluteCost(float* absCost) const
{
    *absCost += cost;
    if ((parentNode != NULL) && !isRoot) parentNode->addToAbsoluteCost(absCost);
}

void SearchTreeNode::traceBackToRoot(PATH_TYPE* stateVector) const
{
    // TODO: inserting at the beginning might slow down the process
    stateVector->insert(stateVector->begin(), state);
    if ((parentNode != NULL) && !isRoot) parentNode->traceBackToRoot(stateVector);
}

void SearchTreeNode::setRoot(bool isNodeRoot)
{
    isRoot = isNodeRoot;
}