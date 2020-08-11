//
// Created by zwb on 2020/6/11.
//

#ifndef NODEGENERATOR_PICTUREDISPLAY_H
#define NODEGENERATOR_PICTUREDISPLAY_H

#include<iostream>

#include<opencv2/opencv.hpp>

#include<hash_map>

using namespace cv;

using namespace std;

class pictureDisplay{
public:
    Mat mat,mat_,dst,img;

    int NodeNumber;

    set<int>Vertex;//这个用来查询

    vector<int>vert;//这个用来遍历

    vector<map<int,int>> vm;//这个用来对节点重新编号，防止序号过大

    vector<int>bestCol;

    pictureDisplay(Mat image);

    ~pictureDisplay();

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


};


#endif //NODEGENERATOR_PICTUREDISPLAY_H
