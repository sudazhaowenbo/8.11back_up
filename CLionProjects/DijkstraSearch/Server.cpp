//
// Created by zwb on 2020/5/27.
//
#include "Server.h"

Server::Server(int port_)
{
    port=port_;

}
Server::~Server()
{

}
int Server::InitialServer()
{

    if (port < 1025 || port > 65535)//0~1024一般给系统使用，一共可以分配到65535
    {
        printf("端口号范围应为1025~65535");
        return -1;
    }

    //1 创建tcp通信socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1) {
        perror("socket failed!\n");
        return -1;
    }
    //2 绑定socket地址
    struct sockaddr_in server_addr = {0};
    server_addr.sin_family = AF_INET;//AF_INET->IPv4
    server_addr.sin_port = htons(port);// server port
    server_addr.sin_addr.s_addr = INADDR_ANY;//server ip (auto set by system)
    int ret = bind(socket_fd, (struct sockaddr *) &server_addr, sizeof(server_addr));
    if (ret == -1) {
        perror("bind failed!\n");
        return -1;
    }
    //3 设置监听队列，设置为可以同时连接5个客户端
    ret = listen(socket_fd, 5);
    if (ret == -1) {
        perror("listen falied!\n");
        return -1;
    }
    printf("server is running!\n");
    struct sockaddr_in client_addr = {0};//用来存放客户端的地址信息
    socklen_t len = sizeof(client_addr);

    //4 等待客户端连接
    new_socket_fd = accept(socket_fd, (struct sockaddr *) &client_addr, &len);
    if (new_socket_fd == -1) {
        perror("accpet error!\n");
    } else {
        printf("IP:%s, PORT:%d [connected]\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    }
}
void Server::ShutDownServer()
{
    close(socket_fd);
    close(new_socket_fd);
}
