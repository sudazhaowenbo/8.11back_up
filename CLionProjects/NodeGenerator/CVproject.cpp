#include<opencv2\opencv.hpp>   
#include<opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\types_c.h>

using namespace std;
using namespace cv;

vector<int> maxAvailNodesInOneCol(Mat matrix);
void AdjacentDiffusion(Mat mat,  int selectedCol, int expansionRatio);
void AccumulatedNodes();
void addEdge(int start, int end, int edge);
void addVertex(int vertex);
//腐蚀
int main()
{
	Mat img = imread("G:\\6.12\\CVproject\\CVproject\\resources\\a.jpg");
	Mat imgGray, result;
	//namedWindow("原始图", WINDOW_NORMAL);
	//imshow("原始图", img);
	cvtColor(img, imgGray, CV_BGR2GRAY);
	Mat out;
	Mat image_eroded_with_3x3_kernel;
	//获取自定义核
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
	//腐蚀操作
	erode(imgGray, image_eroded_with_3x3_kernel, element);

	threshold(image_eroded_with_3x3_kernel, result, 220, 255, CV_THRESH_BINARY);
	
	//namedWindow("grayImg", WINDOW_NORMAL);
	//imshow("grayImg", imgGray);
	namedWindow("腐蚀操作", WINDOW_NORMAL);
	imshow("腐蚀操作", result);

	Mat temImage, dstImage1, dstImage2,out_;
	temImage = result;
	resize(temImage, dstImage1, Size(temImage.cols / 2, temImage.rows / 2), 0, 0, INTER_LINEAR);
	resize(temImage, dstImage2, Size(temImage.cols * 2, temImage.rows * 2), 0, 0, INTER_LINEAR);
	//如果是隔一个点为一个节点则缩小1/2，如果隔两个点为一个节点则缩小1/3，依次类推；
	//实际要按机器人的半径放缩，也就是机器人在地图中占用多少个像素
	//这样放缩以后每个节点就是一个机器人的身位
	//得到的每个节点，通过x,y乘以相应的比例仍旧可以恢复到原来的像素点，再与机器人的坐标系变换后
	//即可得到所有的节点
	//接下去要检索所有列里面空白区域最多的一列，作为基准列，然后相邻扩散。
	//因为已经缩成每个节点了因此也只需要按像素进一步扩散即可
	threshold(dstImage1, out_, 253, 255, CV_THRESH_BINARY);
	vector<int>bestCol;
	bestCol=maxAvailNodesInOneCol(out_);
	AdjacentDiffusion(out_, bestCol.at(0), 3);
	//imshow("缩小", dstImage1);//变小后的点取平均值
	//cvtColor(imgGray, imgRGB, COLOR_GRAY2RGB);
	cout << "cols" << out_.cols << endl;
	cout << "rows" << out_.rows << endl;
	//imshow("二值图", out_);//所有取平均值的点再变为二值化
	//imshow("放大", dstImage2);
	

	waitKey(0);

}

vector<int> maxAvailNodesInOneCol(Mat matrix)
{
	int maxColNum=0, maxNodesInOneCol=0,temp=0;
	vector<int> mycol;
	for (int j = 0; j < matrix.cols; j++) 
	{
		for (int i = 0; i < matrix.rows; i++)//i表示行，j表示列
		{
			//uchar a = 255;
		
			//printf("uc=%d\n", matrix.at<uchar>(0, j));
			if (matrix.at<uchar>(i, j) == 255)
			{
				temp++;
				addVertex(i*matrix.cols+j);
			}
				//cout << "true" << endl;
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
	/*for (int k = 0; k < mycol.size(); k++)
	{
		cout << mycol.at(k)<<endl;
	}
	cout << maxColNum<<endl;*/
	return mycol;
}
void AdjacentDiffusion(Mat mat,int selectedCol,int expansionRatio)
{
	Mat mat_;
	mat_ = mat;
	int distance=3;
	int initialRow=1, initialCol = 0;
	//先纵向检索，间隔为3个大格子，如果节点与节点之间都是通的则添加一条边，如果纵向为3个格子，横向也设为3个大个子，则每隔3个横向大格子检索，这样纵向的就完成了；
	//再横向检索，若列与列之间的大格子相互连通，则添加一条边
	//最好能在图像上画出来边和节点，在机器人执行的时候能够动态变化
	initialCol = selectedCol % distance;
	//纵向检索
	for (int j = initialCol; j < mat.cols; j = j + distance) //逐列扫描，间隔为distance,初始列为selectedCol%distance
	{	
		for (int i = 0; i < mat.rows/distance; i ++)//逐行扫描,间隔为distance
		{
			for (int k = 0; k < distance&&((i*distance+k+1)<mat.rows); k++)//检验从第i个大格子到第i+distance个大格子间是否都为可通行区域，若是则给第i个大格子和第i+distance个大格子之间添加边；
			{
				if (mat.at<uchar>(i*distance + k, j) == 255 && mat.at<uchar>(i*distance + k + 1, j) == 255)
				{
					if (k == distance - 1)
					{
						addEdge((i*distance+ initialRow)*mat.cols+ j, ((i+1)*distance + initialRow -1)*mat.cols+ j,distance);//这里还有一点问题，目前的mat.at是像素值，而我需要的是坐标值。等会改
						addEdge(((i + 1)*distance + initialRow - 1)*mat.cols+ j, (i*distance + initialRow)*mat.cols+ j, distance);//这里的节点有distance个，中心具体选哪个可以进行筛选
						//line(mat_,Point(i*distance + initialRow,j), Point((i + 1)*distance + initialRow - 1, j), Scalar(0, 0, 0), 1, LINE_8);
						circle(mat_, Point(i*distance + initialRow, j), 1, 0);
						circle(mat_, Point(((i + 1)*distance + initialRow - 1), j), 1, 0);
						break;
						
					}
					else
						continue;
				}
				else
					break;
			}
		}
	}
	//横向检索
	for (int i = initialRow; i < mat.rows; i++)//逐行扫描，间隔为distance
	{
		for (int j = 0; j < mat.rows / distance; j++)//逐列扫描,间隔为distance
		{
			for (int k = 0; k < distance && ((j*distance + k + 1) < mat.cols); k++)
			{
				if (mat.at<uchar>(i, j*distance + k) == 255 && mat.at<uchar>(i, j*distance + k + 1) == 255)
				{
					if (k == distance - 1)
					{
						addEdge((i*mat.cols+ j*distance + initialCol), (i*mat.cols + (j + 1)*distance + initialCol), distance);
						addEdge((i*mat.cols + (j + 1)*distance + initialCol), (i*mat.cols + j*distance + initialCol),distance);
						//line(mat_, Point(i, j*distance + initialCol), Point(i, (j + 1)*distance + initialCol), Scalar(0, 0, 0), 1, LINE_4);
						break;
					}
					else
						continue;
				}
				else
					break;
			}
				
		}
	}
	imshow("二值图", mat_);
		
}
void AccumulatedNodes()
{}
void addEdge(int start,int end,int edge)
{
	//cout << "start" << start << "end" << end << "edge" << edge << endl;
}
void addVertex(int vertex)
{}