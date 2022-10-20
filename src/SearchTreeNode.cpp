#include <SearchTreeNode.h>

SearchTreeNode::SearchTreeNode()
{
    childNodes = new std::vector<SearchTreeNode*>(0);
}


SearchTreeNode::SearchTreeNode(const SearchTreeNode &original)
{
    parentNode = original.parentNode;
    //childNodes = original.childNodes;
    childNodes = new std::vector<SearchTreeNode*>;
    copy(original.childNodes->begin(), original.childNodes->end(), back_inserter(*childNodes));
    state = original.state;
}

SearchTreeNode::SearchTreeNode(SearchTreeNode* parent, std::vector<double> stateSpace)
{
    parentNode = parent;
    state = stateSpace;
    childNodes = new std::vector<SearchTreeNode*>(0);
}


void SearchTreeNode::addChild(SearchTreeNode* childNode)
{
    ROS_INFO_STREAM("capacity: "<<childNodes->capacity() << "size: " << childNodes->size());
    childNodes->push_back(childNode);
    ROS_INFO_STREAM("capacity: "<<childNodes->capacity() << "size: " << childNodes->size());
}

void SearchTreeNode::removeChild(SearchTreeNode* childNode)
{
    //std::remove(childVector.begin(), childVector.end(), childNodeID);
}

SearchTreeNode* SearchTreeNode::getParent()
{
    return parentNode;
}

std::vector<SearchTreeNode*> *SearchTreeNode::getChildren()
{
    return childNodes;
}

std::vector<double> SearchTreeNode::getState()
{
    return state;
}

void SearchTreeNode::traceBackToRoot(std::vector<std::vector<double>>* stateVector)
{
    stateVector->insert(stateVector->begin(), state);
    if (parentNode != NULL) parentNode->traceBackToRoot(stateVector);
}

