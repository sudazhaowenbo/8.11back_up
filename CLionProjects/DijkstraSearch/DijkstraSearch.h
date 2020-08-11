//
// Created by zwb on 2020/5/27.
//

#ifndef DIJKSTRASEARCH_DIJKSTRASEARCH_H
#define DIJKSTRASEARCH_DIJKSTRASEARCH_H

#endif //DIJKSTRASEARCH_DIJKSTRASEARCH_H
#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <algorithm>
#include<iostream>

#include<opencv2/opencv.hpp>

#include<hash_map>

using namespace cv;

using namespace std;
using namespace std;
class Robot
{
private:
    vector< vector<pair<int, int> > > adjList;
    vector< pair<int, int> > dist;//first 是权重 second是节点号
    float info[13];
    int FeedBackNodes;

public:
    vector<int>LocalPaths, LocalPath;

    int current_pos;
    int condition;
    int FinalGoal;
    int start;
    int NumberOfNodes;
    vector<int>ReleasedPaths;
    set<int> LocalS, OtherS;//OtherS作地图更新的节点标号
    set<pair< int,int> >SelfInverseEdges,OtherInverseEdges;//self可能没用，other保存其他的所有机器人的边
    //设置每个机器人有顺序的节点序列作为更新Graph的逆边
    //机器人号+节点列表
    //在main函数与类里面筛选出的节点要建立联系，取消掉OtherS作为其他机器人在运动过程中占用的节点，而提取节点与节点之间的边
    //进而修改权重
public:
    Robot(int current_pos_,Mat image);
    ~Robot();
    vector< vector<pair<int, int> > > FormAdjList();
    void DijkstraSP( int &start);
    void PrintShortestPath( int &start, int &goal);
    void AddEdge(int start_node, int end_node, int weight);
    					  //当前机器人的局部路径保存在这里，方便发送时候查找
    set<int>OccupiedNodesOfThisRobot();	  //把当前机器的局部路径和当前位置都保存在这里，方便更新占用信息的时候调用，根据终端机器人传回的节点实时更新
    set<int>UpdateOccupiedNodesOfOtherRobots(set<int> &OccupiedNodesFromOthers);//把每个机器人的占用信息都压入到这里面，用一个循环搜索所有机器人,从其他机器人的LocalPath得到（数据类型为vector)
    void UpdateInverseEdges(vector<int>NodeSequence);
    void clearAllEdges();
    void ChangeGoal(int &new_Goal);
    void ClearOccupiedNodesExceptThisRobot();
    std::vector<std::string> split(std::string str, std::string pattern);
    void SetFeedbackNodesLength(int FeedBackNodes_);
    void SetGoal(int goal_);
    vector<int>PrintLocalPath();
    void SetCurrentPos(int pos);


    //
    Mat mat,mat_,img,img0;

    int NodeNumber;

    set<int>Vertex;//这个用来查询

    vector<int>vert;//这个用来遍历

    map<int,int>Pixel2Num,Num2Pixel;

    map<int,pair<float,float> >Num2Pos;

    vector<int>bestCol;


    vector<int> maxAvailNodesInOneCol();

    void AdjacentDiffusion(  int selectedCol, int expansionRatio);

    void AccumulatedNodes();

    void addEdge(int start, int end, int edge);

    void addVertex(int vertex);

    bool checkEmptySpace(int j,int i);

    bool CheckTopEdge(int xPixel,int yPixel,int distance);

    bool CheckBottomEdge(int xPixel,int yPixel,int distance);

    bool CheckLeftEdge(int xPixel,int yPixel,int distance);

    bool CheckRightEdge(int xPixel,int yPixel,int distance);

    void RotatePicWithShrink();

    Mat RotatePicWithExpansion(double angle);

    Mat CropImg(Mat &src,Mat &dst);

    void rotate_arbitrarily_angle(Mat src, Mat &dst, float angle);

    Mat AutoGenerator();

    Mat Filter(Mat src);

    Mat imgExbansion();

    Mat generateGraph();

    Mat PutTextForNodes(Mat m);

    int PrintNearestNode(float x_m,float y_m);

    void FuncNum2Pos(int Node__,int x_cv,int y_cv,int row,int col);

};