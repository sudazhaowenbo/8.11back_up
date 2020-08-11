//
// Created by zwb on 2020/5/27.
//

#ifndef DIJKSTRASEARCH_SERVER_H
#define DIJKSTRASEARCH_SERVER_H
#endif //DIJKSTRASEARCH_SERVER_H


#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <algorithm>
#include <iostream>
using namespace std;


class Server
{
private:


    struct sockaddr_in client_addr;//用来存放客户端的地址信息
    struct sockaddr_in server_addr;//用来存放服务器地址信息


public:
    int port;
    int socket_fd,new_socket_fd;
    int InitialServer();
    int ret;
    Server(int port_);
    void ShutDownServer();
    ~Server();


};