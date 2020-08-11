
#include "DijkstraSearch.h"
#include "EditDataFromClients.h"
#include "Server.h"
#include "NodeAllocation.h"
#include <unistd.h>
#include <pthread.h>
#include <set>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

using namespace std;
typedef multimap<int, int> multim;//第一个int是节点号，第二个int是机器人号
typedef pair<pair<int, int>, int > doublePair;
#define AmountOfNodes 3

void * recv_msg(void *arg);//接收消息函数声明
void * recv_msg_1(void *arg);
void * send_msg(void *arg);//发送消息函数声明
void * send_msg_1(void *arg);//发送消息函数声明
void * search_path(void *arg);
void * displayImg(void *arg);

static pthread_t send_thread,SearchThread,send_thread2,display_thread;
char received_buf[1024]={0};
char send_buf[1024]={0},send_buf2[1024]={0};
char received_buf1[1024]={0};
bool mapGenerated;
Mat DisplayTheMap;
pthread_mutex_t mutex_send,mutex_rec;

enum {Initial,RequestPath,Pending,ReachFinalGoal};

//现在从机器人处接收来的数据已经能被路径搜索的线程接受并处理了，但要小心处理，包括全局变量的初始化，一定要按照到EditData那个类的形式修改，另外长度也要有所保证，否则数组容易出问题;
//下一就是要让机器人动起来，改变当前的路径点，然后动态去规划他，这一步要把地图处理成设定的N个点，然后每个点要记录位置信息，
int main(int argc, char *argv[])
{
    memset(received_buf1,1,1024);
    memset(received_buf,1,1024);
    //memset(send_buf,0,1024);
    //memset(send_buf2,0,1024);

    strcpy(received_buf,"-1;2;0;0;3;10;1.1;1.2;1.5;2.1;3;5;7;10;");
    strcpy(received_buf1,"-1;2;0;0;3;10;1.1;1.2;1.5;2.1;3;5;7;10;");
    mapGenerated=false;
    int ret,ret2;
    //img2Nodes();
    Server server1(6666),server2(7777);//server1 6666
    //Mat img = imread("/home/zwb/CLionProjects/NodeGenerator/map.pgm");
    //Robot ro(0,img);
    //ro.generateGraph();
    server1.InitialServer();
    server2.InitialServer();
    pthread_mutex_init(&mutex_send, NULL);
    pthread_mutex_init(&mutex_rec, NULL);
    pthread_create(&SearchThread,NULL,&search_path,NULL);
    pthread_create(&display_thread,NULL,&displayImg,NULL);//创建显示线程
    //开启接收线程
    pthread_t recv_thread,recv_thread2;//存放线程id       recv_msg：线程执行的函数，将通信socket：new_socket_fd传递进去
    ret = pthread_create(&recv_thread, NULL, recv_msg, (void *) &server1.new_socket_fd);
    ret2= pthread_create(&recv_thread2,NULL,recv_msg_1,(void *) &server2.new_socket_fd);
    //开启发送线程
    ret = pthread_create(&send_thread, NULL, send_msg, (void *) &server1.new_socket_fd);
    ret2= pthread_create(&send_thread2,NULL,send_msg_1,(void *) &server2.new_socket_fd);//改这句

    if(server1.socket_fd==-1||server1.port<1025||server1.port>65535||server1.ret==-1||server1.new_socket_fd==-1||ret!=0)
    {
        cout<<"Failed to creat server1,program exit"<<endl;
        return 0;
    }
    else {
        pthread_join(send_thread, NULL);
    }

    if(server2.socket_fd==-1||server2.port<1025||server2.port>65535||server2.ret==-1||server2.new_socket_fd==-1||ret!=0)
    {
        cout<<"Failed to creat server1,program exit"<<endl;
        return 0;
    }
    else
        pthread_join(send_thread2,NULL);

    pthread_join(SearchThread,NULL);
    pthread_join(display_thread,NULL);//显示线程
    server1.ShutDownServer();
    server2.ShutDownServer();
    pthread_mutex_destroy(&mutex_rec);
    pthread_mutex_destroy(&mutex_send);
    return 0;
}

void * search_path(void *arg)
{
    //坐标点转化为节点，有以下几种选择1.在接收函数中转化，然后把节点传给搜索函数（不太好）那么需要地图，然后坐标转为像素（需要自己算），像素再转为节点，在像素转节点这一步怕是无法实现，
    // 因为，根本不知道某个节点是第几个节点。
    //2.在搜索函数里面转化，分为两种，一种是把传进来的数据放到EditDataFromClients,本质上和第一种没什么区别，因为也都需要自己重新计算;
    //3.另一种把数据传到robot类里面，这个类里面已经有了,但是在计算邻近节点的时候，如果每次都算的话计算量太大，因此，考虑只在第一次的时候计算邻近节点，之后
    //服务器与机器人通信只通过节点通信，机器人还是发位置坐标给服务器，服务器只管发，不管你执行，机器人如果错了，把错的信息发给服务器即可
    Mat img = imread("/home/zwb/CLionProjects/NodeGenerator/map.pgm");
    EditDataFromClients Editor,Editor1;
    Robot robot[3]={Robot(0,img),Robot(1,img),Robot(2,img)};
    NodeAllocation nodeAlloc;
    DisplayTheMap=robot[0].generateGraph().clone();
    for(int i=0;i<3;i++)
    {
        robot[i].generateGraph();
    }
    mapGenerated=true;
    int start = 0, goal = 13, currentPos,RobotNumber;
    char buf[1024]={0}; memset(buf,0,1024);
    set<int> RobotSet;

    for(int i=0;i<3;i++)
    {
        robot[i].SetGoal(goal-i);
    }
    /*Robot robot0(-1,img),robot1(-1,img),robot2(-1,img),robot3(-1,img),robot4(-1,img);//干脆直接创建多个机器人在服务器上，反正占用内存也不多，起始点为0;

    DisplayTheMap=robot0.generateGraph().clone();
    robot1.generateGraph();
    robot2.generateGraph();
    robot3.generateGraph();
    robot4.generateGraph();
    mapGenerated=true;
    int start = 0, goal = 13, currentPos,RobotNumber;
    char buf[1024]={0}; memset(buf,0,1024);
    robot0.SetGoal(goal);robot1.SetGoal(goal-1),robot2.SetGoal(goal-2),robot3.SetGoal((goal-3)),robot4.SetGoal(goal-4);
    set<int> RobotSet;
     */
    string SendString;
    ostringstream oss;
    //假设服务器收到如下字符串char info x;2;0;0;3;10;1.1;1.2;1.5;2.1;3;5;7;10;
    //其中第一位为机器人序号，第二位为机器人开启关闭标志位（Awake=1,OffLine=0)，第三位为机器人行进状态OffLine(BrokeDown,Silent)Awake(Pending,RequirePath,ReachFinalGoal)，第四位为当前位置（节点），第五位为全局目标，
    //第六位为X方向速度，第7位为Y方向速度，第8位为Z方向速度，第9位为转向四元数，第10位为第一个局部路径节点，第11位为第二个为局部路径节点，第12位为第三个局部路径节点
    //目前共13位，第0位暂时保留；
    //因此要将char型字符串Info解包成相应的信息，在Dijkstra里面处理（因为是每个机器人的私有变量），先在主函数接收以后，检验机器人序号，然后把这些信息赋值给相应的机器人；

    strcpy(buf,"-1;2;0;0;3;10;1.1;1.2;1.5;2.1;3;5;7;10;");//机器人发来的数据只有一串自己的信息，服务器发过去可以是一整串所有机器人的信息，机器人根据自己序号提取信息

    Editor.GetData(buf);
    Editor1.GetData(buf);

    //RobotNumber=Editor.PrintRobotSequence();

    //在set里面找到机器人序号,则说明已经创建过了,不管;如果没找到说明还没有,则根据标志位创建一个,用new创建在堆里面,
    //方便后续如果这个机器人退出给他删除,原则上,只要机器人连接上服务器就给他创建,直到他退出的时候,发送退出标志位
    if(!RobotSet.count(RobotNumber))
    {
        //这部分算是客户端传来的信息给做路径规划的服务器一个初始状态
        //Robot *NewVariable(RobotNumber)=new Robot(start,goal);//后期完善的时候可以用这种方法,现在先把效果跑出来，只要机器人连接上即可
        currentPos=Editor.PrintRobotCurrentNode();
        RobotSet.insert(RobotNumber);
    }
    //机器人发来的数据只有一串自己的信息，服务器发过去可以是一整串所有机器人的信息，机器人根据自己序号提取信息
    strcpy(buf,"-1;2;1;1;0;13;1.1;1.2;1.5;2.1;3;5;7;10;");

    Editor.GetData(buf);

    RobotNumber=Editor.PrintRobotNumber();

    //Search Path For connected robots until all goal reached ,cout x robot has arrived
    while (1) {
        cout << received_buf << endl;
        if (received_buf != 0)
            Editor.GetData(received_buf);//get data from clients
        //这里收到位置以后需要转换成节点号
        oss.str("");
        SendString.clear();

        //节点搜索算法
        if (Editor.isRobotOnline() && Editor.PrintRobotNumber() == 0)//Editor代表robot0
        {
            //根据机器人发送的情况规划，如果机器人自己知道在第几个节点，则进入RequestPath，如果不知道则进入Initial
            if (Editor.PrintWorkingState() == Initial)//在线状态0
            {
                currentPos = robot[0].PrintNearestNode(Editor.PrintRobotPosX(), Editor.PrintRobotPosY());
                robot[0].SetCurrentPos(currentPos);
                oss << SendString << Editor.PrintRobotNumber() << " " << true << " " << "|";
                SendString = oss.str();
                SendString.copy(send_buf, SendString.length(), 0);
                send_buf[SendString.length()] = '\0';
            } else if (Editor.PrintWorkingState() == RequestPath)//在线状态1
            {
                //pthread_mutex_lock(&mutex_rec);
                robot[0].LocalPaths.clear();
                robot[0].SetGoal(Editor.PrintRobotFinalGoal());
                currentPos = Editor.PrintRobotCurrentNode();//currentPos由Robot类里面算出最近的节点，再赋值
                //currentPos =robot0.PrintNearestNode(Editor.x,Editor.y);
                robot[0].SetCurrentPos(currentPos);
                robot[0].OtherS.clear();
                robot[0].clearAllEdges();//清空搜索图的占用信息
                //先清楚其他机器人的占用状态（因为可能已经更新了），然后压入其他机器人新的状态，包括机器人占用的边（用于在搜索时禁用）和
                //第一轮搜索时，机器人并不知道其他机器人的行进方向，但是当有机器人占用了某个方向以后便需要将这个信息加入到搜索图中，而不再把机器人的占用点作
                //临时节点,因此只能把其他机器人占用的节点作为临时障碍物考虑，
                //第二轮搜索时，
                //或者认为，每个被占用的节点都是有方向的，如果占用不移动，则占用所有方向，如果占用
                //这些认为是离线的机器人可以不用
                //robot0.UpdateOccupiedNodesOfOtherRobots(robot1.LocalS);
                //robot0.UpdateOccupiedNodesOfOtherRobots(robot2.LocalS);
                //robot0.UpdateOccupiedNodesOfOtherRobots(robot3.LocalS);
                //这些是其他机器人占用的逆边
                for (int m = 0; m < 3; m++) {
                    if (robot[m].LocalPaths.empty() || m == 0)
                        continue;
                    else {
                        robot[0].UpdateInverseEdges(robot[m].LocalPaths);
                    }
                }

                //搜索路径
                robot[0].DijkstraSP(currentPos);
                //robot0.PrintLocalPath();
                //这里加一句转换的代码，把每个LocalPaths里面的元素转为x,y坐标
/*
                oss<<SendString<<Editor.PrintRobotNumber()<<" "<<true<<" "<<"|";
                for(int i=0;i<robot[0].LocalPaths.size()&&i<=2;i++)//这里要改节点号变成坐标点
                {
                    float x_map,y_map;
                    float vx=10.5,vy=10.7,w=10.9;
                    x_map=robot[0].Num2Pos.find(robot[0].LocalPaths.at(i))->second.first;
                    y_map=robot[0].Num2Pos.find(robot[0].LocalPaths.at(i))->second.second;
                    oss<<SendString<<robot[0].LocalPaths.at(i)<<" "<<x_map<<" "<<y_map<<" "
                    <<vx<<" "<<vy<<" "<<w<<" "<<"|";
                }
                //pthread_mutex_unlock(&mutex_rec);
                oss<<SendString<<"#";
                SendString=oss.str();
                SendString.copy(send_buf,SendString.length(),0);
                send_buf[SendString.length()]='\0';
                cout << "robot"<<oss.str() << endl;
*/
            } else if (Editor.PrintWorkingState() == Pending)//在线状态2
            {
                //机器人正在行进，跳过
                cout << "Robot is pending" << endl;
            } else if (Editor.PrintWorkingState() == ReachFinalGoal)//在线状态3
            {
                //机器人到达全局路径终点
                cout << "Robot has arrived the final goal" << endl;
            } else {
                cout << "Something gets wrong" << endl;
            }
        } else {
            //等等写离线情况
        }

        oss.str("");
        SendString.clear();

        Editor1.GetData(received_buf1);
        if (Editor1.isRobotOnline() && Editor1.PrintRobotNumber() == 1) {
            if (Editor1.PrintWorkingState() == Initial)//在线状态0
            {
                currentPos = robot[1].PrintNearestNode(Editor1.PrintRobotPosX(), Editor1.PrintRobotPosY());
                robot[1].SetCurrentPos(currentPos);
                oss << SendString << Editor1.PrintRobotNumber() << " " << true << " " << currentPos << " " << "|";
                SendString = oss.str();
                SendString.copy(send_buf2, SendString.length(), 0);
                send_buf2[SendString.length()] = '\0';
            } else if (Editor1.PrintWorkingState() == RequestPath)//在线状态1
            {
                //pthread_mutex_lock(&mutex_rec);
                robot[1].LocalPaths.clear();

                robot[1].SetGoal(Editor1.PrintRobotFinalGoal());
                currentPos = Editor1.PrintRobotCurrentNode();
                robot[1].SetCurrentPos(currentPos);
                robot[1].OtherS.clear();
                robot[1].clearAllEdges();
                /*
                robot[1].UpdateOccupiedNodesOfOtherRobots(robot[0].LocalS);
                robot[1].UpdateOccupiedNodesOfOtherRobots(robot[2].LocalS);
                robot[1].UpdateOccupiedNodesOfOtherRobots(robot[3].LocalS);
                robot[1].DijkstraSP(currentPos);
                robot[1].PrintLocalPath();
                 */
                //这些是其他机器人占用的逆边
                for (int m = 0; m < 3; m++) {
                    if (robot[m].LocalPaths.empty() || m == 1)
                        continue;
                    else {
                        robot[1].UpdateInverseEdges(robot[m].LocalPaths);
                    }
                }

                //robot[1].UpdateInverseEdges(robot[0].LocalPaths);
                //robot[1].UpdateInverseEdges(robot[2].LocalPaths);
                //robot[1].UpdateInverseEdges(robot[3].LocalPaths);
                //搜索路径
                robot[1].DijkstraSP(currentPos);
                /*
                oss<<SendString<<Editor1.PrintRobotNumber()<<" "<<true<<" "<<"|";
                for(int i=0;i<robot[1].LocalPaths.size()&&i<=2;i++)
                {
                    float x_map,y_map;
                    float vx=10.5,vy=10.7,w=10.9;
                    x_map=robot[1].Num2Pos.find(robot[1].LocalPaths.at(i))->second.first;
                    y_map=robot[1].Num2Pos.find(robot[1].LocalPaths.at(i))->second.second;
                    oss<<SendString<<robot[1].LocalPaths.at(i)<<" "<<x_map<<" "
                    <<y_map<<" "<<vx<<""<<vy<<" "<<w<<" "<<"|";
                }

                //pthread_mutex_unlock(&mutex_rec);

                oss<<SendString<<"#";
                SendString=oss.str();
                SendString.copy(send_buf2,SendString.length(),0);
                send_buf2[SendString.length()]='\0';
                cout << "robot"<<oss.str() << endl;
                 */
            } else if (Editor1.PrintWorkingState() == Pending)//在线状态2
            {
                //机器人正在行进，跳过
                cout << "Robot is pending" << endl;
            } else if (Editor1.PrintWorkingState() == ReachFinalGoal)//在线状态3
            {
                //机器人到达全局路径终点
                cout << "Robot has arrived the final goal" << endl;
            } else {
                cout << "Something gets wrong" << endl;
            }
        } else {
            //等等写离线情况
        }
        sleep(2);

        vector<vector<int>> robotAlloc;

        //节点分配算法
        robotAlloc.clear();
        nodeAlloc.OriginalNodes.clear();
        for (int i = 0; i < 3; i++) {
            if (!robot[i].LocalPaths.empty())
            {
                robotAlloc.push_back(robot[i].LocalPaths);
            }
        }
        sleep(1);
        nodeAlloc.OriginalNodes=robotAlloc;
        nodeAlloc.alloc(robotAlloc);

        //每个机器人搜索到的节点都压入mp里面去筛选，包括正在执行的和准备发送的
    }

        //节点发送算法
        //把机器人类的每个对象按数组的形式计入，然后N个机器人循环更新数据发送（要根据机器人的当前状况，也就是收到的Editor的标志位该机器人是否需要新的节点序列
        //若是不需要直接continue,需要则发送
/*
        for(int j=0;j<3;j++)
        {
            switch(j)
            {
                case 0:
                    {
                    oss.str("");
                    SendString.clear();
                    if (Editor1.PrintWorkingState()!=RequestPath)
                        break;
                    oss << SendString << Editor.PrintRobotNumber() << " " << true << " " << "|";
                    for (int i = 0; i < robotAlloc[0].size() && i <= 2; i++)//这里要改节点号变成坐标点
                    {//现在是最多发三个点，后面根据需要更改
                        float x_map, y_map;
                        float vx = 10.5, vy = 10.7, w = 10.9;
                        x_map = robot[0].Num2Pos.find(robotAlloc[0].at(i))->second.first;
                        y_map = robot[0].Num2Pos.find(robotAlloc[0].at(i))->second.second;
                        oss << SendString << robotAlloc[0].at(i) << " " << x_map << " " << y_map << " "
                            << vx << " " << vy << " " << w << " " << "|";
                    }
                    //pthread_mutex_unlock(&mutex_rec);
                    oss << SendString << "#";
                    SendString = oss.str();
                    SendString.copy(send_buf, SendString.length(), 0);
                    send_buf[SendString.length()] = '\0';
                    cout << "robot" << oss.str() << endl;
                }

                case 1:
                {
                    oss.str("");
                    SendString.clear();
                    if(Editor1.PrintWorkingState()!=RequestPath)
                        break;
                    oss << SendString << Editor1.PrintRobotNumber() << " " << true << " " << "|";
                    for (int i = 0; i < robotAlloc[1].size() && i <= 2; i++)
                    {
                        float x_map, y_map;
                        float vx = 10.5, vy = 10.7, w = 10.9;
                        x_map = robot[1].Num2Pos.find(robotAlloc[1].at(i))->second.first;
                        y_map = robot[1].Num2Pos.find(robotAlloc[1].at(i))->second.second;
                        oss << SendString << robotAlloc[1].at(i) << " " << x_map << " "
                            << y_map << " " << vx << "" << vy << " " << w << " " << "|";
                    }

                    //pthread_mutex_unlock(&mutex_rec);

                    oss << SendString << "#";
                    SendString = oss.str();
                    SendString.copy(send_buf2, SendString.length(), 0);
                    send_buf2[SendString.length()] = '\0';
                    cout << "robot1" << oss.str() << endl;
                    break;
                }
                default:
                {
                    cout<<"robot "<<j<<"does not update path"<<endl;
                }
            }
*/


}

void * recv_msg(void *arg)
{
    int *socket_fd = (int *)arg;//通信的socket
    while(1)
    {

        char buf[1024] = {0};
        //pthread_mutex_lock(&mutex_rec);
        read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
        //pthread_mutex_unlock(&mutex_rec);
        strcpy(received_buf,buf);
            //printf("receive msg:%s\n", buf);

        if(strncmp(buf, "exit", 4) == 0 || strcmp(buf, "") == 0)
        {
            //通知主线程。。。
            printf("GOODBYE\n");
            bzero(buf, 1024);
            pthread_cancel(SearchThread);
            pthread_cancel(send_thread);
            pthread_cancel(display_thread);//退出显示线程
            break;//退出
        }

    }
    return NULL;
}
void * recv_msg_1(void *arg)
{
    int *socket_fd = (int *)arg;//通信的socket
    while(1)
    {

        char buf[1024] = {0};
        //pthread_mutex_lock(&mutex_rec);
        read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
        //pthread_mutex_unlock(&mutex_rec);
        strcpy(received_buf1,buf);
            //printf("receive msg:%s\n", buf);

        if(strncmp(buf, "exit", 4) == 0 || strcmp(buf, "") == 0)
        {
            //通知主线程。。。
            printf("GOODBYE\n");
            bzero(buf, 1024);
            pthread_cancel(send_thread);
            pthread_cancel(SearchThread);
            pthread_cancel(display_thread);//退出显示线程
            pthread_cancel(send_thread2);//退出发送线程
            break;//退出
        }

    }
    return NULL;
}
void * send_msg(void *arg)
{
    char buf[1024] = {0};
    memset(send_buf,0,1024);

    //strcpy(buf,"0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;");
    int *socket_fd=(int *)arg;
    while(1)
    {
        //写入节点的处理语句
        strcpy(buf,send_buf);
        sleep(1);
        pthread_mutex_lock(&mutex_send);
        write(*socket_fd,buf,strlen(buf));
        pthread_mutex_unlock(&mutex_send);
        sleep(1);
        int i=0;
        cout<<"send msg"<<endl;
        while(i!=1023)
        {
            cout<<buf[i];
            i=i+1;
            //cout<<str.length()<<endl;
        }
        i=0;

        sleep(4);
        /*
        if(strcmp(buf, "exit") == 0 || strcmp(buf, "") == 0)
        {
            break;//退出
        }
        */
    }

    pthread_exit(NULL);
}
void * send_msg_1(void *arg)
{
    pthread_mutex_t mutex_send1;
    char buf[1024] = {0};
    memset(send_buf2,0,1024);
    //strcpy(send_buf2,"-1;2;1;1;0;13;1.1;1.2;1.5;2.1;3;5;7;10;");
    strcpy(send_buf2,"3|1 0.1 0.1 1.1 1.1 3|2 0.2 0.2 2.2 2.2 3|#");
    //strcpy(send_buf2,"3;|;1;0.1;0.1;1.1;1.1;3;| 2;0.2;0.2;2.2;2.2;3;| #");
    //strcpy(buf,"0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;");
    int *socket_fd=(int *)arg;
    while(1)
    {
        //写入节点的处理语句
        strcpy(buf,send_buf2);
        sleep(1);
        //pthread_mutex_lock(&mutex_send1);
        write(*socket_fd,buf,strlen(buf));
        //pthread_mutex_unlock(&mutex_send1);
        sleep(1);
        int i=0;
        while(i!=1023)
        {
            cout<<buf[i];
            i=i+1;
            //cout<<str.length()<<endl;
        }
        i=0;

        sleep(4);
        /*
        if(strcmp(buf, "exit") == 0 || strcmp(buf, "") == 0)
        {
            break;//退出
        }
        */
    }

    pthread_exit(NULL);
}
void * displayImg(void *arg)
{
    while(1)
    {
        if(mapGenerated&&!DisplayTheMap.empty())
        {
            namedWindow("节点标记图", WINDOW_NORMAL);
            imshow("节点标记图", DisplayTheMap);
            waitKey(0);
        }
        else
        {
            sleep(2);
        }
    }


}
