//
// Created by zwb on 2020/8/10.
//

#ifndef DIJKSTRASEARCH_NODEALLOCATION_H
#define DIJKSTRASEARCH_NODEALLOCATION_H

#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <algorithm>

using namespace std;
typedef multimap<int, int> multim;//第一个int是节点号，第二个int是机器人号
typedef pair<pair<int, int>, int > doublePair;
class NodeAllocation {
    multim mp;
    int RobotNumber;
public:
    vector< vector<int> >OriginalNodes,AdjustedNodes;
    void alloc(vector<vector<int> >s);
};


#endif //DIJKSTRASEARCH_NODEALLOCATION_H
