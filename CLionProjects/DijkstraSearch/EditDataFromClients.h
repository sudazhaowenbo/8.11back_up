//
// Created by zwb on 2020/5/28.
//

#ifndef DIJKSTRASEARCH_EDITDATAFROMCLIENTS_H
#define DIJKSTRASEARCH_EDITDATAFROMCLIENTS_H


#include <stdio.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h> //atoi()
#include <vector>
#include <string>
using namespace std;

class EditDataFromClients {

private:
    char buf[1024];
    vector<string> subString;
    vector<float> ClientData;//所有的浮点数据都存在这里
    vector<int>employedLocalPath;
    vector<int>releasedLocalPath;
public:
    EditDataFromClients();
    ~EditDataFromClients();

    void GetData(char buf_[]);

    vector<string> SubString();

    vector<string> split(string str, string pattern);

    vector<float> ConvertSTof();

    int PrintRobotNumber();

    bool isRobotOnline();

    int PrintRobotCurrentNode();

    int PrintRobotFinalGoal();

    vector<float>PrintRobotVelocity();

    vector<int>PrintLocalPath();

    vector<int>PrintEmployedPath();

    vector<int>PrintReleasedPath();

    int PrintWorkingState();

    float PrintRobotPosX();

    float PrintRobotPosY();
};


#endif //DIJKSTRASEARCH_EDITDATAFROMCLIENTS_H
