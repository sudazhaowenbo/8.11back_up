//
// Created by zwb on 2020/5/28.
//

#include "EditDataFromClients.h"
EditDataFromClients::EditDataFromClients()
{
    buf[1024]={0};
}
EditDataFromClients::~EditDataFromClients(){}

void EditDataFromClients::GetData(char buf_[]) //给类中的变量buf赋值，用于字符串处理操作
{
    memset(buf,0,1024);
    strcpy(buf,buf_);
    SubString();
    ConvertSTof();
/*    PrintRobotSequence();
    isRobotOnline();
    PrintRobotCurrentNode();
    PrintRobotFinalGoal();
    PrintRobotVelocity();
    PrintLocalPath();
    PrintEmployedPath();
    PrintReleasedPath();
*/
}
vector<string> EditDataFromClients::SubString()//返回N串以;分割的字串，每串都是string类型,同时s也被赋值
{

    string myString;

    string pattern=";";

    myString.clear();

    myString.assign(buf,1024);

    vector<string> v;

    subString.clear();

    subString=split(myString,pattern);

    return v;
}
vector<string> EditDataFromClients::split(string str, string pattern)
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

vector<float> EditDataFromClients::ConvertSTof()
{
    ClientData.clear();

    for(int i=0;i<subString.size();i++)
    {
        ClientData.push_back(atof(subString[i].data()));
    }

};

int EditDataFromClients::PrintRobotNumber()
{
    int robotSeq;
    robotSeq=ClientData.at(1);
    return robotSeq;
}

bool EditDataFromClients::isRobotOnline()
{
    bool RobotCondition;
    RobotCondition=ClientData.at(2);
    return RobotCondition;
}

int EditDataFromClients::PrintWorkingState()
{
    int state;
    state=ClientData.at(3);
    return state;
}

int EditDataFromClients::PrintRobotCurrentNode()
{
    int current_node;
    current_node=ClientData.at(4);
    return current_node;
}
float EditDataFromClients::PrintRobotPosX()
{
    return ClientData.at(10);
}
float EditDataFromClients::PrintRobotPosY()
{
    return ClientData.at(11);
}

int EditDataFromClients::PrintRobotFinalGoal()
{
    int FinalGoalNode;
    FinalGoalNode=ClientData.at(5);
    return FinalGoalNode;
}

vector<float>EditDataFromClients::PrintRobotVelocity()
{
    vector<float>v;
    v.clear();
    float X_Velocity,Y_Velocity,Z_Velocity,Orientation;
    X_Velocity=ClientData.at(6);
    Y_Velocity=ClientData.at(7);
    Z_Velocity=ClientData.at(8);
    Orientation=ClientData.at(9);
    v.push_back(X_Velocity);
    v.push_back(Y_Velocity);
    v.push_back(Z_Velocity);
    v.push_back(Orientation);
    return v;
}

vector<int>EditDataFromClients::PrintLocalPath()
{
    //这部分要考虑怎么处理局部路径，包括占用的和释放的,一种办法是通过正负号来判断，节点从1开始,正号代表被该机器人占用,负号表示节点已释放
    vector<int>NodesOfLocal;
    NodesOfLocal.clear();
    employedLocalPath.clear();
    releasedLocalPath.clear();
    for(int i=10;i<ClientData.size();i++)
    {
        NodesOfLocal.push_back(ClientData.at(i));
        if(ClientData.at(i)>0)
            employedLocalPath.push_back(ClientData.at(i));
        else
            releasedLocalPath.push_back(ClientData.at(i));
    }
    return NodesOfLocal;

}

vector<int>EditDataFromClients::PrintEmployedPath()
{

    return employedLocalPath;
}

vector<int>EditDataFromClients::PrintReleasedPath()
{
    return releasedLocalPath;
}

