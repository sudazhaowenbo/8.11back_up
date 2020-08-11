
#include"DijkstraSearch.h"

Robot::Robot(int current_pos_,Mat image)
{
    img0=image;
    //mat_=mat.clone();
    NodeNumber=0;
    //adjList=FormAdjList();
    current_pos = current_pos_;
    OccupiedNodesOfThisRobot();
}
Mat Robot::generateGraph()
{
    Mat final;
    Mat Label;
    cout<<img0.cols<<""<<img0.rows<<endl;
    img=imgExbansion().clone();
    final=AutoGenerator();//到这里节点已经生成，经验证每个节点之间的连通信息都正确
    Label=PutTextForNodes(final);//到这里主要是为了给每个点的位置编号，并显示在图片上
    //namedWindow("节点生成图", WINDOW_NORMAL);
    //imshow("节点生成图", final);
    //namedWindow("节点标记图", WINDOW_NORMAL);
    //imshow("节点标记图", Label);
    //waitKey(0);
    //namedWindow("二值图", WINDOW_NORMAL);
    //imshow("二值图", final);//所有取平均值的点再变为二值化
    //自动生成图
    //adjList=FormAdjList();	//刚开始是创建好的图，如果后期是需要按照情况更改的图的话，建了添加顶点和边的语句，用于更新,执行完后得到adjList
    return Label;
}
Mat Robot::PutTextForNodes(Mat m)
{
    Mat m_;
    m_=m.clone();
    map<int,int>::iterator it;
    int xPix,yPix;
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX,0.5, 1.0, 0,1,8);
    for(int i=0;i<Num2Pixel.size();i++)
    {
        stringstream s;
        string str;
        it=Num2Pixel.find(i);
        s<<i;
        s>>str;
        if(it!=Num2Pixel.end())
        {
            if(m_.rows>m_.cols)
            {
                xPix=it->second%mat.rows;
                yPix=it->second/mat.rows;
                cv::putText(m_,str,Point(xPix,yPix-1),CV_FONT_HERSHEY_COMPLEX_SMALL,0.5,Scalar (255 ,225 ,255),0.02,8);
            }
            else
            {
                xPix=it->second/mat.cols;
                yPix=it->second%mat.cols;
                cv::putText(m_,str,Point(xPix,yPix-1),CV_FONT_HERSHEY_COMPLEX_SMALL,0.5,Scalar (255 ,225 ,255),0.02,8);
            }

        }

    }
    return m_;
}

Robot::~Robot() {};
vector< vector<pair<int, int> > > Robot::FormAdjList()
{
    // We have 7 vertices, so initialize 7 rows.
    const int n = NodeNumber;

    for (int i = 0; i < n; i++)
    {
        // Create a vector to represent a row, and add it to the adjList.
        vector<pair<int, int> > row;
        adjList.push_back(row);
    }

    /*adjList[0].push_back(make_pair(1, 2));//0->1 edge:2
    adjList[0].push_back(make_pair(2, 3));  //0->2 edge:3

    adjList[1].push_back(make_pair(0, 2));
    adjList[1].push_back(make_pair(5, 1));

    adjList[2].push_back(make_pair(0, 3));
    adjList[2].push_back(make_pair(5, 2));

    adjList[3].push_back(make_pair(1, 4));
    adjList[3].push_back(make_pair(4, 1));
    adjList[3].push_back(make_pair(6, 2));

    adjList[4].push_back(make_pair(3, 1));
    adjList[4].push_back(make_pair(5, 2));
    adjList[4].push_back(make_pair(6, 1));

    adjList[5].push_back(make_pair(1, 1));
    adjList[5].push_back(make_pair(2, 2));
    adjList[5].push_back(make_pair(4, 2));
    adjList[5].push_back(make_pair(6, 2));

    adjList[6].push_back(make_pair(3, 2));
    adjList[6].push_back(make_pair(4, 1));
    adjList[6].push_back(make_pair(5, 2));
    adjList[6].push_back(make_pair(7, 2));
    // Our graph is now represented as an adjacency list. Return it.

    adjList[7].push_back(make_pair(8, 2));
    adjList[7].push_back(make_pair(9, 3));

    adjList[8].push_back(make_pair(7, 2));
    adjList[8].push_back(make_pair(9, 1));

    adjList[9].push_back(make_pair(7, 3));
    adjList[9].push_back(make_pair(6, 2));
    adjList[9].push_back(make_pair(10, 2));

    adjList[10].push_back(make_pair(11, 4));
    adjList[10].push_back(make_pair(9, 1));
    adjList[10].push_back(make_pair(8, 2));

    adjList[11].push_back(make_pair(5, 1));
    adjList[11].push_back(make_pair(12, 2));
    adjList[11].push_back(make_pair(13, 1));


    adjList[12].push_back(make_pair(6, 2));
    adjList[12].push_back(make_pair(10, 2));
    adjList[12].push_back(make_pair(13, 2));

    adjList[13].push_back(make_pair(14, 2));
    adjList[13].push_back(make_pair(11, 1));
    adjList[13].push_back(make_pair(12, 2));
    adjList[14].push_back(make_pair(15, 2));
    adjList[15].push_back(make_pair(16, 2));
*/
    return adjList;
}

void Robot::DijkstraSP( int &start)
{

    cout << "\nGetting the shortest path from " << start << " to all other nodes.\n";
    //vector<pair<int, int> > dist; // First int is dist, second is the previous node.
    int flag = false;
    // Initialize all source->vertex as infinite.
    int n = adjList.size();
    dist.clear();
    for (int i = 0; i < n; i++)
    {
        dist.push_back(make_pair(1000000007, i)); // Define "infinity" as necessary by constraints.
    }

    // Create a PQ.
    priority_queue<pair<int, int>, vector< pair<int, int> >, greater<pair<int, int> > > pq;

    // Add source to pq, where distance is 0.
    pq.push(make_pair(start, 0));
    dist[start] = make_pair(0, start);
    // While pq isn't empty...
    while (pq.empty() == false)
    {
        flag = false;
        // Get min distance vertex from pq. (Call it u.)
        int u = pq.top().first;
        pq.pop();
        //检测set里面是否包含和u相等的值，即该节点是否已被访问过
        //思路是：节点遇到临时障碍物则跳过，当然，但凡机器人在线便不认为是临时障碍物（也就是每个机器人都是有任务的，
        // 不允许停留在通道上，无任务停留在通道中视为出现故障需要处理，因此在设计的时候需要在地图上设置任务点），边遇到逆边则跳过
        //节点和逆边各自保存为一个集合，在集合中搜索
        //set<int> otherS,set<pair<int,int> > InverseEdge
        /*if (!OtherS.empty())
        {
            for (auto &i : OtherS)
            {
                if (i == u)
                    flag = true;
            }
        }
        if (flag == true)
            continue;
            */
        // Visit all of u's friends. For each one (called v)....
        for (int i = 0; i < adjList[u].size(); i++)
        {
            //如果u后面的节点v是某个机器人占用的逆向边，则进入下一次循环，即InverseEdge.find(make_pair(u,v))==true,continue,weight不更新

            int v = adjList[u][i].first;//u是节点号，i是邻接的几条边，first是相邻节点，second是节点之间的权值
            int weight = adjList[u][i].second;//本来应该是邻接边的权值，现在因为被占用，所以边值无穷大

            pair<int,int> nodePair;
            nodePair.first=u;
            nodePair.second=v;

            if(OtherInverseEdges.find(nodePair)!=OtherInverseEdges.end())//表示逆边已被占用，跳过此次循环
            {
                continue;
            }

            // If the distance to v is shorter by going through u...
            if (dist[v].first > dist[u].first + weight)//更新到节点距离
            {
                // Update the distance of v.
                dist[v].first = dist[u].first + weight;
                // Update the previous node of v.
                dist[v].second = u;//到v的点是u
                // Insert v into the pq.
                pq.push(make_pair(v, dist[v].first));
            }
        }

    }
    //ClearOccupiedNodesExceptThisRobot();
    PrintShortestPath(start,FinalGoal);

}

void Robot::PrintShortestPath( int &start,int &goal)
{
    LocalPaths.clear();
    cout << "\nPrinting the shortest paths for node " << start << ".\n";
    for (int i = 0; i < dist.size(); i++)
    {
        if (i == FinalGoal)
        {
            cout << "The distance from node " << start << " to node " << i << " is: " << dist[i].first << endl;
            int currnode = i;//i是终点
            LocalPaths.push_back(currnode);
            cout << "The path is: " << currnode;
            while (currnode != start)
            {
                if (dist[i].first > 999999)//100万个节点，如果1米一个节点，正方形空场地，则可以1Km*1Km的范围
                {
                    cout << "No path from node " << start << " to node " << i << endl;
                    break;
                }
                else
                {
                    currnode = dist[currnode].second;
                    cout << " <- " << currnode;
                    LocalPaths.push_back(currnode);
                    /*if (LocalPaths.size() >= 4)
                        break;*/
                }
            }
        }
        else
            continue;
        cout << endl << endl;
        reverse(LocalPaths.begin(), LocalPaths.end());

    }
    //OccupiedNodesOfThisRobot();
}

void Robot::AddEdge(int start_node,int end_node,int weight)
{
    int NodeNumber_start,NodeNumber_end;
    map<int,int>::iterator it_start,it_end;
    it_start=Pixel2Num.find(start_node);
    it_end=Pixel2Num.find(end_node);
    if(it_start!=Pixel2Num.end())
    {
        NodeNumber_start=Pixel2Num.find(start_node)->second;
        if(it_end!=Pixel2Num.end())
        {
            NodeNumber_end=Pixel2Num.find(end_node)->second;
            adjList[NodeNumber_start].push_back(make_pair(NodeNumber_end, weight));//后期使用，暂时不需要
            adjList[NodeNumber_end].push_back(make_pair(NodeNumber_start,weight));//用的是双向边，如果某条边是单向的，不妨把某个方向的权重设置为无穷大
        }
    } else
    {
        cout<<"No edge between "<<start_node<<"and"<<end_node<<endl;
    }


}

set<int>Robot::OccupiedNodesOfThisRobot ()//当前机器人的路径和当前位置，根据远程机器人传回的释放节点实时更新，这部分的占用信息是要一直保存的，
//只有机器人有权对他进行清除，分为以下几种情况进行清除：
//1.机器人到达指定点，则全部清除后，计算新的目标点；2.机器人释放某些局部点则清除部分点；3.机器人到达最终目标点，则清除全部点，只保留最终目标点
{
    LocalS.clear();
    LocalS.insert(current_pos);
    for (auto &i : LocalPaths)		//保存当前机器人的路径和位置
    {
        LocalS.insert(i);
    }
    if (!ReleasedPaths.empty())		//收到终点机器人传回的释放节点信息
    {
        for (auto &i : ReleasedPaths)
        {
            LocalS.erase(i);
        }
    }
    return LocalS;
}
set<int>Robot::UpdateOccupiedNodesOfOtherRobots(set<int> &OccupiedNodesFromOthers)//用来更新其他机器人占用的信息
{
    //OtherS.clear();		//这里遗留一个问题，就是怎样去处理占用节点的动态信息，最好应该是当这个机器人路径生成以后，再clear，或者达到终点，否则一直计算
    //OtherS只在要进行计算Dijkstra的时候才赋值，其他时候保持空的状态，计算完以后再清除；前设条件是机器人在不需要路径时，不做dijkstra计算
    for (auto &i : OccupiedNodesFromOthers)
    {
        OtherS.insert(i);
    }
    return OtherS;
}
void Robot::UpdateInverseEdges(vector<int> NodeSequence)
{
    vector<int> edges=NodeSequence;
    if(NodeSequence.size()<2)
    {
        cout<<"There is no inverse Path for this robot"<<endl;
    }
    else
    {
        for(int i=1;i<edges.size();i++)
        {
            OtherInverseEdges.insert(pair<int,int>(edges.at(i),edges.at(i-1)));
            cout<<edges.at(i);
        }

    }


}
void Robot::clearAllEdges()
{
    SelfInverseEdges.clear();
    OtherInverseEdges.clear();
}
vector<int>Robot::PrintLocalPath()//重新调整局部路径的节点长度，并返回
{
    LocalPath.clear();
    if (LocalPaths.size() > 3)
    {
        for (int i = 0; i <= 3; i++)
        {
            LocalPath.push_back(LocalPaths[i]);
        }
        return LocalPath;
    }
    else
        return LocalPath;

}
void Robot::ChangeGoal(int &new_Goal)
{
    FinalGoal = new_Goal;
}

void Robot::ClearOccupiedNodesExceptThisRobot()
{
    OtherS.clear();
}

std::vector<std::string> Robot::split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;

    str += pattern;//扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i < size; i++) {
        pos = str.find(pattern, i);
        if (pos < size) {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}

void Robot::SetGoal(int goal_) {
    FinalGoal =goal_;
}
void Robot::SetCurrentPos(int pos)
{
    current_pos=pos;
}

void Robot::SetFeedbackNodesLength(int FeedBackNodes_)
{
    FeedBackNodes=FeedBackNodes_;
}

//

Mat Robot::AutoGenerator()
{
    //遗留问题,旋转后的坐标与真实环境坐标匹配;以及检验节点与节点之间是否有边，难点在于旋转之后，某些点并无法击中像素，有可能在几个像素的中间
    //暂且搁置，等到真实环境下能跑出一定效果以后再回头考虑。
    //原则上，在不影响实际运行效果的前提下，尽可能让视觉效果更佳，如果视觉效果实在无法保证，则通过录像的方式表达。
    //下一步，在垂直地图上进行测试，包括产生的节点坐标，和构建邻接表。
    Mat imgGray, result,dst1,dst2,dst,img1;
    img1=img.clone();
    dst1=img.clone();
    //bestCol=maxAvailNodesInOneCol();
    rotate_arbitrarily_angle(img1,dst1,90);
    //mat_=dst1;
    mat=dst1.clone();
    mat_=dst1.clone();

    AdjacentDiffusion(1, 3);
    //rotate_arbitrarily_angle(mat_,dst,-90);
    //dst2=CropImg(img, dst);
    //dst3=Filter(dst2);
    //GaussianBlur(dst2,dst3 ,0, 100, 15);

    return mat_;
}

Mat Robot::Filter(Mat src)
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

vector<int> Robot::maxAvailNodesInOneCol()
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

void Robot::RotatePicWithShrink()
{}

void Robot::AdjacentDiffusion( int selectedCol,int expansionRatio)
{

    int node,xPixel,yPixel,pixProduct;
    int distance=15;//这里的distance需要特别注意，是两个节点的中心的间距，如果错了就无法定位到末端节点，
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
                    pixProduct=x+y*mat_.rows;
                    FuncNum2Pos(vert.size(),yPixel,xPixel,mat_.rows,mat_.cols);
                    Vertex.insert(pixProduct);//这里Vertex保存了所有的节点
                    vert.push_back(x+y*mat_.rows);
                }
                else
                {
                    xPixel=x;
                    yPixel=y;
                    pixProduct=x*mat_.cols+y;
                    FuncNum2Pos(vert.size(),yPixel,xPixel,mat_.rows,mat_.cols);
                    Vertex.insert(pixProduct);//这里Vertex保存了所有的节点
                    vert.push_back(x*mat_.cols+y);

                }
                addVertex(pixProduct);
            }
            x=x+distance;
        }
        y=y+distance;//当把这句改为+1时，纵向变成一列，因此y是列
    }
    cout<<"nodeNumber"<<NodeNumber<<endl;
    adjList=FormAdjList();//节点生成好以后就把节点传入到邻接表里面，以免等会添加边的时候出错
    for(int k=0;k<vert.size();k++)
    {
        node=vert.at(k);
        if(mat_.rows>mat_.cols)
        {
            xPixel=node%mat_.rows;
            yPixel=node/mat_.rows;
            if(CheckTopEdge(xPixel,yPixel,distance))
            {
                AddEdge(node,xPixel+(yPixel-distance)*mat_.rows,distance);
                //AddEdge(xPixel+(yPixel-distance)*mat_.rows,distance,node);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel-n,xPixel)[0]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[1]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[2]=0;
                }
            }
            if(CheckLeftEdge(xPixel,yPixel,distance))
            {
                AddEdge(node,xPixel-distance+yPixel*mat_.rows,distance);//这里上下左右要小心
                //AddEdge(xPixel-distance+yPixel*mat_.rows,distance,node);
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
                //把这两个node的坐标积map以后，添加边，也就是对map搜索两次找到两个node的second数据，再添加边
                AddEdge(node,xPixel*mat_.cols+yPixel-distance,distance);
                //AddEdge(xPixel*mat_.cols+yPixel-distance,node,distance);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel-n,xPixel)[0]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[1]=255;
                    mat_.at<Vec3b>(yPixel-n,xPixel)[2]=0;
                }
            }
            if(CheckLeftEdge(xPixel,yPixel,distance))
            {
                AddEdge(node,(xPixel-distance)*mat_.cols+yPixel,distance);
                //AddEdge((xPixel-distance)*mat_.cols+yPixel,node,distance);
                for(int n=1;n<distance;n++)
                {
                    mat_.at<Vec3b>(yPixel,xPixel-n)[0]=0;
                    mat_.at<Vec3b>(yPixel,xPixel-n)[1]=255;
                    mat_.at<Vec3b>(yPixel,xPixel-n)[2]=255;
                }
            }

        }
    }
    cout<<"Graph Constructed"<<endl;
}

void Robot::AccumulatedNodes()
{}

void Robot::addEdge(int start,int end,int edge)
{

    //cout << "start" << start << "end" << end << "edge" << edge << endl;
}

void Robot::addVertex(int vertex)
{

    Pixel2Num.insert(pair<int,int>(vertex,Pixel2Num.size()));
    Num2Pixel.insert(pair<int,int>(Num2Pixel.size(),vertex));


}

bool Robot::checkEmptySpace(int i,int j)//i列，j行
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

bool Robot::CheckTopEdge(int xPixel,int yPixel,int distance)
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

bool Robot::CheckBottomEdge(int xPixel,int yPixel,int distance)
{

}

bool Robot::CheckLeftEdge(int xPixel,int yPixel,int distance)
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

bool Robot::CheckRightEdge(int xPixel,int yPixel,int distance)
{

}

Mat Robot::CropImg(Mat &src,Mat &dst)
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

void Robot::rotate_arbitrarily_angle(Mat src, Mat &dst, float angle)
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
    Rect rect(x, y, targetSize.width, targetSize.height);
    dst = Mat(dst, rect);
    dst  ;
}

Mat Robot::imgExbansion()
{
    Mat result;
    Mat image_eroded_with_3x3_kernel;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(10, 10)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
    //腐蚀操作
    erode(img0, image_eroded_with_3x3_kernel, element);
    result=image_eroded_with_3x3_kernel;
    //threshold(image_eroded_with_3x3_kernel, result, 220, 255, CV_THRESH_BINARY);
    //namedWindow("grayImg", WINDOW_NORMAL);
    //imshow("grayImg", imgGray);
    //namedWindow("腐蚀操作", WINDOW_NORMAL);
    //imshow("腐蚀操作", result);

    Mat temImage, dstImage1, dstImage2,out_;
    temImage = result;
    resize(temImage, dstImage1, Size(temImage.cols / 1, temImage.rows / 1), 0, 0, INTER_LINEAR);
    resize(temImage, dstImage2, Size(temImage.cols * 2, temImage.rows * 2), 0, 0, INTER_LINEAR);
    //namedWindow("二值图", WINDOW_NORMAL);
    out_=dstImage1;
    //threshold(dstImage1, out_, 200, 255, CV_THRESH_BINARY);
    vector<int>bestCol;
    //pictureDisplay picDis(out_);
    //bestCol=picDis.maxAvailNodesInOneCol();
    // picDis.AdjacentDiffusion(bestCol.at(0), 3);//注意传递的参数要小心，从c改过来的c++保留了一部分c的写法
    //imshow("缩小", dstImage1);//变小后的点取平均值
    //cvtColor(imgGray, imgRGB, COLOR_GRAY2RGB);
    //Mat final;
    //final=picDis.pictureDisplay::AutoGenerator();
    //cout << "cols" << out_.cols << endl;
    //cout << "rows" << out_.rows << endl;


    //namedWindow("二值图", WINDOW_NORMAL);
    //imshow("二值图", final);//所有取平均值的点再变为二值化
    //imshow("放大", dstImage2);


    //waitKey(0);
    return out_;
}

void Robot::FuncNum2Pos(int Node__, int x_cv,int y_cv,int row,int col)
{
    float Resolution=0.05, X_Origin=-10.0, Y_Origin=-10.0;
    float x_map,y_map;
    pair<float,float>s_;
    x_map=(row-x_cv)*Resolution+X_Origin;
    y_map=(col-y_cv)*Resolution+Y_Origin;
    s_=pair<float,float> (x_map,y_map) ;
    Num2Pos.insert(pair<int,pair<float,float> >(Node__,s_));
}

int Robot::PrintNearestNode(float x_m,float y_m)
{
    int x_cv,y_cv;
    int row,col;
    int NearestNode;
    float Origin_x=-10,Origin_y=-10;
    float Resolution=0.05;
    int distance=mat_.rows*mat_.cols;
    map<int,int>::iterator iter;
    x_cv=mat_.rows-(x_m-Origin_x)/Resolution;
    y_cv=mat_.cols-(y_m-Origin_y)/Resolution;

    if(mat_.rows>mat_.cols)
    {
        for (iter = Num2Pixel.begin(); iter != Num2Pixel.end(); iter++)
        {
            row = iter->second / mat_.rows;
            col = iter->second % mat_.cols;
            if(pow(x_cv - row, 2) + pow(y_cv - col, 2)<distance)
            {
                distance = pow(x_cv - row, 2) + pow(y_cv - col, 2);
                NearestNode=iter->first;
            }
        }
    }
    else
    {
        for (iter = Num2Pixel.begin(); iter != Num2Pixel.end(); iter++)
        {
            row=iter->second%mat_.rows;
            col=iter->second/mat_.cols;
            if(pow(x_cv - row, 2) + pow(y_cv - col, 2)<distance)
            {
                distance=pow(x_cv-row,2)+pow(y_cv-col,2);
                NearestNode=iter->first;
            }
        }
    }
    return NearestNode;
}

