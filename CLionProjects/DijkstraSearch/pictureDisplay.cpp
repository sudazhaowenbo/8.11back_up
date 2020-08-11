//
// Created by zwb on 2020/6/11.
//

#include "pictureDisplay.h"
#include <hash_map>
using namespace std;
pictureDisplay::pictureDisplay(Mat image)
{
    img=image;
    //mat_=mat.clone();
    NodeNumber=0;
    /*
    vector<vector<int> > A;//第一个A是节点编号，第二个B是节点的X，Y坐标    vector<int> B;
    vector<int>B;
    B.push_back(0);
    B.push_back(1);

    A.push_back(B);
    B.clear();
    B.push_back(3);
    B.push_back(4);
    A.push_back(B);
     */
}

Mat pictureDisplay::AutoGenerator()
{
    //遗留问题,旋转后的坐标与真实环境坐标匹配;以及检验节点与节点之间是否有边，难点在于旋转之后，某些点并无法击中像素，有可能在几个像素的中间
    //暂且搁置，等到真实环境下能跑出一定效果以后再回头考虑。
    //原则上，在不影响实际运行效果的前提下，尽可能让视觉效果更佳，如果视觉效果实在无法保证，则通过录像的方式表达。
    //下一步，在垂直地图上进行测试，包括产生的节点坐标，和构建邻接表。
    Mat imgGray, result,dst1,dst2,dst3,dst,img1;
    img1=img.clone();
    dst1=img.clone();
    //bestCol=maxAvailNodesInOneCol();
    //rotate_arbitrarily_angle(img1,dst1,30);
    //mat_=dst1;
    mat=dst1.clone();
    mat_=dst1.clone();
    //namedWindow("原始图2", WINDOW_NORMAL);
    //imshow("原始图2", mat_);
    AdjacentDiffusion(1, 3);
    //rotate_arbitrarily_angle(mat_,dst,-90);
    //dst2=CropImg(img, dst);
    //dst3=Filter(dst2);
    //GaussianBlur(dst2,dst3 ,0, 100, 15);
    return mat_;
}

pictureDisplay::~pictureDisplay()
{}

Mat pictureDisplay::Filter(Mat src)
{
    Mat AfterFilter;
    AfterFilter=src;
    unsigned char blue,green,red;
    for(int i=0;i<src.rows;i++)
    {
        for(int j=0;j<src.cols;j++)
        {
            blue=AfterFilter.at<Vec3b>(i,j)[0];
            green=AfterFilter.at<Vec3b>(i,j)[1];
            red=AfterFilter.at<Vec3b>(i,j)[2];
            if(blue==205&&green==205&&red==205)
                continue;
            else if(blue==254&&green==254&&red==254)
                continue;
            else if(blue=0&&green==0&&red==0)
                continue;
            else if(blue<=205&&green<=205&&red<=205)
            {
                AfterFilter.at<Vec3b>(i,j)[0]=205;
                AfterFilter.at<Vec3b>(i,j)[1]=205;
                AfterFilter.at<Vec3b>(i,j)[2]=205;
            }
            else if(blue>205&&green>205&&red>205)
            {
                AfterFilter.at<Vec3b>(i,j)[0]=254;
                AfterFilter.at<Vec3b>(i,j)[1]=254;
                AfterFilter.at<Vec3b>(i,j)[2]=254;
            }

            else if(blue<205&&green>205&&red>205)
            {
                AfterFilter.at<Vec3b>(i,j)[0]=0;
                AfterFilter.at<Vec3b>(i,j)[1]=255;
                AfterFilter.at<Vec3b>(i,j)[2]=255;
            }
            else if(blue>205&&green<205&&red>205)
            {
                AfterFilter.at<Vec3b>(i,j)[0]=255;
                AfterFilter.at<Vec3b>(i,j)[1]=0;
                AfterFilter.at<Vec3b>(i,j)[2]=255;
            }
            else if(blue>205&&green>205&&red<205)
            {
                AfterFilter.at<Vec3b>(i,j)[0]=255;
                AfterFilter.at<Vec3b>(i,j)[1]=255;
                AfterFilter.at<Vec3b>(i,j)[2]=0;
            }

            else
                continue;
        }
    }

    return AfterFilter;
}

vector<int> pictureDisplay::maxAvailNodesInOneCol()
{
    int maxColNum=0, maxNodesInOneCol=0,temp=0;
    vector<int> mycol;
    for (int j = 0; j < mat.cols; j++)
    {
        for (int i = 0; i < mat.rows; i++)//i表示行，j表示列
        {

            if (checkEmptySpace(i,j) == true)
            {
                temp++;
                addVertex(i*mat.cols+j);
            }
        }
        if (maxNodesInOneCol < temp)
        {
            maxNodesInOneCol = temp;
            maxColNum = j;
            mycol.clear();
            mycol.push_back(j);
        }
        else if (maxNodesInOneCol == temp)
        {
            mycol.push_back(j);
        }
        temp = 0;
    }

    return mycol;
}

void pictureDisplay::RotatePicWithShrink()
{}

void pictureDisplay::AdjacentDiffusion( int selectedCol,int expansionRatio)
{

    int node,xPixel,yPixel;
    int distance=20;//这里的distance需要特别注意，是两个节点的中心的间距，如果错了就无法定位到末端节点，
    int initialRow=1+selectedCol % distance, initialCol = 0;
    //先纵向检索，间隔为3个大格子，如果节点与节点之间都是通的则添加一条边，如果纵向为3个格子，横向也设为3个大个子，则每隔3个横向大格子检索，这样纵向的就完成了；
    //再横向检索，若列与列之间的大格子相互连通，则添加一条边
    //最好能在图像上画出来边和节点，在机器人执行的时候能够动态变化
    initialCol = selectedCol % distance;
    for(int y=initialRow;y<mat_.rows;)
    {
        for(int x=initialCol;x<mat_.cols;)
        {
            if(checkEmptySpace(x,y)==true)//x表示列号，y表示行号，第四象限
            {
                mat_.at<Vec3b>(y,x)[2]=255;
                mat_.at<Vec3b>(y,x)[1]=0;
                mat_.at<Vec3b>(y,x)[0]=255;
                NodeNumber++;
                if(mat_.rows>mat_.cols)//因为地图长宽很多时候不一样，时而长大于宽，时而宽大于长，故而当用短的的边去划分节点号的时候肯定会重复，所以要用长的边来划分
                {
                    xPixel=x;
                    yPixel=y;
                    Vertex.insert(x+y*mat_.rows);//这里Vertex保存了所有的节点
                    vert.push_back(x+y*mat_.rows);
                }
                else
                {
                    xPixel=x;
                    yPixel=y;
                    Vertex.insert(x*mat_.cols+y);//这里Vertex保存了所有的节点
                    vert.push_back(x*mat_.cols+y);
                }
            }
            x=x+distance;
        }
        y=y+distance;//当把这句改为+1时，纵向变成一列，因此y是列
    }
    for(int k=0;k<vert.size();k++)
    {
        node=vert.at(k);
        if(mat_.rows>mat_.cols)
        {
            xPixel=node%mat_.rows;
            yPixel=node/mat_.rows;
            if(CheckTopEdge(xPixel,yPixel,distance))
            {
                addEdge(node,yPixel-distance+xPixel*mat_.cols,distance);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel-n,xPixel)[0]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[1]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[2]=0;
                }
            }
            if(CheckLeftEdge(xPixel,yPixel,distance))
            {
                addEdge(node,yPixel-distance+xPixel*mat_.cols,distance);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel,xPixel-n)[0]=0;
                    mat_.at<Vec3b>(yPixel,xPixel-n)[1]=255;
                    mat_.at<Vec3b>(yPixel,xPixel-n)[2]=255;
                }
            }
        }
        else
        {
            xPixel=node/mat_.cols;
            yPixel=node%mat_.cols;
            if(CheckTopEdge(xPixel,yPixel,distance))
            {
                addEdge(node,yPixel-distance+xPixel*mat_.cols,distance);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel-n,xPixel)[0]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[1]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[2]=0;
                }
            }
            if(CheckLeftEdge(xPixel,yPixel,distance))
            {
                addEdge(node,yPixel-distance+xPixel*mat_.cols,distance);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel,xPixel-n)[0]=0;
                    mat_.at<Vec3b>(yPixel,xPixel-n)[1]=255;
                    mat_.at<Vec3b>(yPixel,xPixel-n)[2]=255;
                }
            }

        }
    }

}

void pictureDisplay::AccumulatedNodes()
{}

void pictureDisplay::addEdge(int start,int end,int edge)
{

    //cout << "start" << start << "end" << end << "edge" << edge << endl;
}

void pictureDisplay::addVertex(int vertex)
{

    map<int,int>m;
    m.insert(pair<int,int>(vm.size(),vertex));
    vm.push_back(m);
}

bool pictureDisplay::checkEmptySpace(int i,int j)//i列，j行
{
    unsigned char blue,green,red;
    blue=mat.at<Vec3b>(j,i)[0];//()里面是（行，列）
    green=mat.at<Vec3b>(j,i)[1];
    red=mat.at<Vec3b>(j,i)[2];

    if(blue=254&&green==254&&red==254)
        return true;
    else
        return false;
}

bool pictureDisplay::CheckTopEdge(int xPixel,int yPixel,int distance)
{
    bool flag=0;
    int blue,green,red;
    if(yPixel-distance<0)
        return false;
    for(int i=0;i<distance;i++)
    {
        //从距当前节点为1的位置开始判断
        yPixel=yPixel-1;
        blue=mat.at<Vec3b>(yPixel,xPixel)[0];
        green=mat.at<Vec3b>(yPixel,xPixel)[1];
        red=mat.at<Vec3b>(yPixel,xPixel)[2];
        if(blue==254&&green==254&&red==254)
        {
            flag=true;
            continue;
        }
        else
        {
            flag=false;

            /*blue;
            red;
            green;
            cout<<"blue"<<blue<<"green"<<green<<"red"<<red<<endl;
             */
            break;
        }
    }
    return flag;
}

bool pictureDisplay::CheckBottomEdge(int xPixel,int yPixel,int distance)
{

}

bool pictureDisplay::CheckLeftEdge(int xPixel,int yPixel,int distance)
{
    bool flag=0;
    int blue,green,red;
    if(xPixel-distance<0)
        return false;
    for(int i=0;i<distance;i++)
    {
        //从距当前节点为1的位置开始判断
        xPixel=xPixel-1;
        blue=mat.at<Vec3b>(yPixel,xPixel)[0];
        green=mat.at<Vec3b>(yPixel,xPixel)[1];
        red=mat.at<Vec3b>(yPixel,xPixel)[2];
        if(blue==254&&green==254&&red==254)
        {
            flag=true;
            continue;
        }
        else
        {
            flag=false;

            /*blue;
            red;
            green;
            cout<<"blue"<<blue<<"green"<<green<<"red"<<red<<endl;
             */
            break;
        }
    }
    return flag;
}

bool pictureDisplay::CheckRightEdge(int xPixel,int yPixel,int distance)
{

}

Mat pictureDisplay::CropImg(Mat &src,Mat &dst)
{
    int left, right, top, bottom;
    int test;
    bool flag = true;
    Mat crop;
    for (int i = 0; i < dst.cols&&flag; i++)
    {
        for (int j = 0; j < dst.rows; j++)
        {
            if (dst.at<Vec3b>(j, i)[0] != 205)
                continue;
            else
            {
                top = j-1;
                left = i-1;
                flag = false;
                break;
            }
        }
    }
    flag = true;

    Rect rect(left, top, src.cols, src.rows);
    crop = Mat(dst, rect);
    //cout<<"test top left point"<<int(crop.at<Vec3b>(0,0)[0])<<endl;
    return crop;
}

void pictureDisplay::rotate_arbitrarily_angle(Mat src, Mat &dst, float angle)
{
    float radian = (float)(angle / 180.0 * CV_PI);

    //填充图像
    int maxBorder = (int)(max(src.cols, src.rows)* 1.414); //即为sqrt(2)*max
    int dx = (maxBorder - src.cols) / 2;
    int dy = (maxBorder - src.rows) / 2;
    copyMakeBorder(src, dst, dy, dy, dx, dx, BORDER_CONSTANT);

    //旋转
    Point2f center((float)(dst.cols / 2), (float)(dst.rows / 2));
    Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);//求得旋转矩阵
    warpAffine(dst, dst, affine_matrix, dst.size(), INTER_AREA,BORDER_CONSTANT ,Scalar(0,0,0));
    //warpAffine(dst, dst, affine_matrix, dst.size());
    //计算图像旋转之后包含图像的最大的矩形
    float sinVal = abs(sin(radian));
    float cosVal = abs(cos(radian));
    Size targetSize((int)(src.cols * cosVal + src.rows * sinVal),
                    (int)(src.cols * sinVal + src.rows * cosVal));

    //剪掉多余边框
    int x = (dst.cols - targetSize.width) / 2;
    int y = (dst.rows - targetSize.height) / 2;
    //Rect rect(x, y, targetSize.width, targetSize.height);
    //dst = Mat(dst, rect);
    dst  ;
}
