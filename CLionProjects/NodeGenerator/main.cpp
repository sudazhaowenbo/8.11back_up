#include<iostream>

#include<opencv2/opencv.hpp>

#include"pictureDisplay.h"

int main() {

    Mat img = imread("/home/zwb/CLionProjects/NodeGenerator/map.pgm");
    Mat imgGray, result,dst1,dst2,dst3,dst;;
    //pictureDisplay ig(img);
    //ig.rotate_arbitrarily_angle(img, dst1, 45);
    //ig.rotate_arbitrarily_angle(dst1, dst, -45);
    //dst2=ig.CropImg(img, dst);
    //cout << "cols" << img.cols << endl;
    //cout << "rows" << img.rows << endl;
    //Point2f src_center(img.rows / 2.0,img.cols / 2.0);
    //Mat warp_mat = getRotationMatrix2D(src_center,45,1.0);
    //Mat dst_img;
    //warpAffine(img,dst_img,warp_mat,img.size());
    //ig.RotatePicWithExpansion(45);
    //ig.AdjacentDiffusion(1,1);
    //ig.RotatePicWithExpansion(135);
    //namedWindow("原始图1", WINDOW_NORMAL);
    //imshow("原始图1", img);

    //namedWindow("二值图", WINDOW_NORMAL);
    //cvtColor(img, imgGray, CV_BGR2GRAY);
    Mat out;
    Mat image_eroded_with_3x3_kernel;
    //获取自定义核
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
    //腐蚀操作
    erode(img, image_eroded_with_3x3_kernel, element);
    result=image_eroded_with_3x3_kernel;
    //threshold(image_eroded_with_3x3_kernel, result, 220, 255, CV_THRESH_BINARY);

    //namedWindow("grayImg", WINDOW_NORMAL);
    //imshow("grayImg", imgGray);
    namedWindow("腐蚀操作", WINDOW_NORMAL);
    imshow("腐蚀操作", result);

    Mat temImage, dstImage1, dstImage2,out_;
    temImage = result;
    resize(temImage, dstImage1, Size(temImage.cols / 0.5, temImage.rows / 0.5), 0, 0, INTER_LINEAR);
    resize(temImage, dstImage2, Size(temImage.cols * 2, temImage.rows * 2), 0, 0, INTER_LINEAR);
    //namedWindow("二值图", WINDOW_NORMAL);

    //如果是隔一个点为一个节点则缩小1/2，如果隔两个点为一个节点则缩小1/3，依次类推；
    //实际要按机器人的半径放缩，也就是机器人在地图中占用多少个像素
    //这样放缩以后每个节点就是一个机器人的身位
    //得到的每个节点，通过x,y乘以相应的比例仍旧可以恢复到原来的像素点，再与机器人的坐标系变换后
    //即可得到所有的节点
    //接下去要检索所有列里面空白区域最多的一列，作为基准列，然后相邻扩散。
    //因为已经缩成每个节点了因此也只需要按像素进一步扩散即可
    out_=dstImage1;
    //threshold(dstImage1, out_, 200, 255, CV_THRESH_BINARY);
    vector<int>bestCol;
    pictureDisplay picDis(out_);
   //bestCol=picDis.maxAvailNodesInOneCol();
   // picDis.AdjacentDiffusion(bestCol.at(0), 3);//注意传递的参数要小心，从c改过来的c++保留了一部分c的写法
    //imshow("缩小", dstImage1);//变小后的点取平均值
    //cvtColor(imgGray, imgRGB, COLOR_GRAY2RGB);
    Mat final;
    final=picDis.pictureDisplay::AutoGenerator();
    cout << "cols" << out_.cols << endl;
    cout << "rows" << out_.rows << endl;

    namedWindow("二值图", WINDOW_NORMAL);
    imshow("二值图", final);//所有取平均值的点再变为二值化
    //imshow("放大", dstImage2);


    waitKey(0);
}
