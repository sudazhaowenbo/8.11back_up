//
// Created by zwb on 2020/8/10.
//

#include "NodeAllocation.h"

// multimaptest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <algorithm>

using namespace std;
typedef multimap<int, int> multim;//第一个int是节点号，第二个int是机器人号
typedef pair<pair<int, int>, int > doublePair;
void NodeAllocation::alloc(vector<vector<int> >s)
{
    int RobotNum=OriginalNodes.size();
    multim mp;
    multim::iterator first,end,iter,it;
    vector<int> robot[RobotNum];
    //vector压入一定要保持这个顺序，错了顺序就全错了
    for(int i=0;i<RobotNum;i++)
    {
        for(int j=0;j<OriginalNodes.at(i).size();j++)
        robot[i].push_back(OriginalNodes[i].at(j));
    }
    /*
    robot[0].push_back(3);
    robot[0].push_back(4);
    robot[0].push_back(5);
    robot[0].push_back(6);
    //robot[0].pop_back();

    robot[1].push_back(5);
    robot[1].push_back(6);
    robot[1].push_back(7);
    robot[1].push_back(8);

    robot[2].push_back(6);
    robot[2].push_back(7);
    robot[2].push_back(4);
    robot[2].push_back(10);
    */
    for (int j = 0; j < RobotNum; j++)
    {
        for (int i = 0; i < robot[j].size(); i++)
        {
            //pair<int, int>s(j, i);
            mp.insert(pair<int,int>(robot[j].at(i),j ));//第二位机器人号
        }
    }
    /*
    mp.insert(doublePair( pair<int,int>(3,4), 3));
    mp.insert(doublePair(pair<int, int>(3, 4), 2));

    mp.insert(doublePair(pair<int, int>(3, 4), 2));
    mp.insert(doublePair(pair<int, int>(3, 5), 0));
    mp.insert(doublePair(pair<int, int>(5, 6), 2));

    mp.insert(pair<pair<int, int>, int>(pair<int, int>(4, 5), 0));

    mp.insert(pair<pair<int, int>, int>(pair<int, int>(6, 7), 2));
    */
    //first = mp.lower_bound(pair<int, int>(3, 4));
    //end = mp.upper_bound(pair<int, int>(3, 4));

    int temp=mp.begin()->first;
    vector<doublePair> vp;
    vector<int> vi;

    vp.clear();
    vi.clear();

    for (iter = mp.begin(); iter != mp.end(); iter++)
    {
        temp = iter->first;
        vi.push_back(temp);
        if (iter == mp.begin() && mp.count(temp) == 1)//multimap的初始值
        {
            continue;
        }
        else if (mp.count(temp) == 1)//前值（临时值temp）与当前值不同，更新临时值，继续下一次循环
        {
            temp = iter->first;
            continue;
        }
        else//找到占用同一个节点的机器人，开始处理multimap和相应的vector
        {

            if (find(vi.begin(), vi.end(), temp) == vi.end())
            {
                continue;
            }
            //first = mp.lower_bound(temp);
            //end = mp.upper_bound(temp);
            //cout << "k" << endl;
            for (it = mp.lower_bound(temp); it != mp.upper_bound(temp); it++)//找到相同节点以及对应的机器人号
            {
                //cout << "9" << endl;
                int nodeSequence;
                for (int k = 0; k < robot[it->second].size(); k++)//it->first是节点名，it->second机器人编号
                {
                    if (it->first == robot[it->second].at(k))
                    {
                        nodeSequence = k;
                        vp.push_back(doublePair(pair<int, int>(it->second, it->first), nodeSequence));//第一位机器人编号，第二位节点名，第三位节点在该机器人中的序号，
                    }
                }
                //先判断该节点在各自机器人中的第几个节点，然后筛选节点序列最低的保留，其余的则将该节点之后的所有节点清除，vector用pop，Multimap用erase
            }
        }
    }
    for (int m = 1,q= 0; m < vp.size(); m++,q++)
    {
        int node;
        if (vp.at(m).second > vp.at(q).second)//下一个机器人在该节点的序号要更大,大的删除
        {
            node = m;
        }
        else
        {
            node = q;
        }
        //把下一个机器人在该节点及其之后的节点在vector和mp里面都除去
        int tmpRobotNumber = vp.at(node).first.first;
        int currentNodeOfThisRobot = vp.at(node).first.second;

        for (int n = 0; n < robot[tmpRobotNumber].size(); n++)//遍历该机器人的所有节点，找到该节点位置
        {
            if (currentNodeOfThisRobot == robot[tmpRobotNumber].at(n))//找到该节点的位置为n
            {
                for (int p = n; p < robot[tmpRobotNumber].size(); p++)//该机器人从该节点往后的所有节点都移除
                {
                    //int currentNodeOfThisRobot= vp.at(node).first.first;
                    robot[tmpRobotNumber].pop_back();
                }
                break;
            }
        }
    }
    //temp = iter->first;

    //iter= mp.lower_bound(pair<int, int>(3, 4));
    //mp.erase(iter);
    //mp.find(pair<int, int>(3, 4));
    //first = mp.lower_bound(pair<int, int>(3, 4));
    //end = mp.upper_bound(pair<int, int>(3, 4));
    cout << "after tackled" << endl;
    for (int i = 0; i < RobotNum; i++)
    {
        for (int j = 0; j < robot[i].size(); j++)
        {
            cout <<i<<"--"<< robot[i].at(j) << endl;
        }
    }
    cout << "all nodes" << endl;
    for (iter = mp.begin(); iter != mp.end(); iter++)
    {
        //全部处理完后，输出最终的multimap

        cout << iter->first << "--" << iter->second << endl;
        //	cout << iter->first.first << "--" << iter->first.second << "--" << iter->second << endl;
    }
    std::cout << "Hello World!\n";
}


