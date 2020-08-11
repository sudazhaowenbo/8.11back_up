#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h> //atoi()
#include <pthread.h>
#include <string>
#include <vector>
#include <iostream>
using namespace std;
static pthread_t send_thread;
//客户端的发送功能要放在子线程里面
void * recv_msg(void *arg);//接收消息函数声明
void * send_msg(void *arg);
std::vector<std::string> split(std::string str, std::string pattern);
struct myData
{
	float x[10];
	float y[10];
	float z[10];
	float w[10];
	
};
	static struct myData mData; 
int main(int argc, char *argv[])
{
	//判断参数个数是否匹配

	int port = 7777;//从命令行接收参数
	if( port<1025 || port>65535 )//0~1024一般给系统使用，一共可以分配到65535
	{
		printf("端口号范围应为1025~65535");
		return -1;
	}
 
	//1 创建tcp通信socket
	int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(socket_fd == -1)
	{
		perror("socket failed!\n");
	}
 
	//2 连接服务器
	struct sockaddr_in server_addr = {0};//服务器的地址信息
	server_addr.sin_family = AF_INET;//IPv4协议
	server_addr.sin_port = htons(port);//服务器端口号
	server_addr.sin_addr.s_addr = inet_addr("10.70.176.205");//设置服务器IP
	int ret = connect(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
	if(ret == -1)
	{
		perror("connect failed!\n");
	}
	else
	{
		printf("connect server successful!\n");
	}
	//开启接收线程
	pthread_t recv_thread;//存放线程id       recv_msg：线程执行的函数，将通信socket：new_socket_fd传递进去
	ret = pthread_create(&recv_thread, NULL, recv_msg, (void*)&socket_fd);
	if(ret != 0)
	{
		printf("开启接收线程失败\n");
	}
 	ret= pthread_create(&send_thread,NULL,send_msg,(void*)&socket_fd);
	if(ret != 0)
	{
		printf("开启发送线程失败\n");
	}

 	pthread_join(send_thread,NULL);
	//pthread_join(send_thread,NULL);
	//4 关闭通信socket
	close(socket_fd);
 
	return 0;
}
 
void * send_msg(void *arg)
{
	char buf[1024] = {0};
	strcpy(buf,"-1;1;1;1;1;3;10;1;2;3;4;1.1;1.2;0;0;0;0;");//0.5;-0.5;1.0;1.0;-2;0.5;1;7;8;9;10;11;-1;-2;-3;
	int *socket_fd=(int *)arg;

	while(1)
	{
		//写入节点的处理语句
		sleep(2);
		write(*socket_fd,buf,strlen(buf));
	}
	pthread_exit(NULL); 
}
//接收线程所要执行的函数 接收消息
void * recv_msg(void *arg)
{
	char buf[1024]={0};
	int *socket_fd=(int *)arg;
	std::string myString(buf,1024);
	std::vector<std::string> v;
	std::string pattern=";";
	while(1)
	{
		read(*socket_fd, buf, sizeof(buf));//阻塞，等待接收消息
		myString.assign(buf,1024);
		//int sign=1;
		v=split(myString,pattern);
		int i=0;
		while(i!=1023)
		{
			cout<<buf[i];
			i=i+1;
			//cout<<str.length()<<endl;
		}
		cout<<endl;
		i=0;
		/*int counter[10]={0};
		for(int i=0;i<=15;i++)//提取数字
		{
			int j=0;
			switch(i%4)
			{
				case(0):
				{
					mData.x[counter[0]++]=atof(v[i].data());
					std::cout<<"x[0]:"<<mData.x[0]<<std::endl;
					std::cout<<"x[1]:"<<mData.x[1]<<std::endl;
					std::cout<<"x[2]:"<<mData.x[2]<<std::endl;
					std::cout<<"x[3]:"<<mData.x[3]<<std::endl;
					break;
				}
				case(1):
				{
					mData.y[counter[1]++]=atof(v[i].data());
					std::cout<<"y[0]:"<<mData.y[0]<<std::endl;
					std::cout<<"y[1]:"<<mData.y[1]<<std::endl;
					std::cout<<"y[2]:"<<mData.y[2]<<std::endl;
					std::cout<<"y[3]:"<<mData.y[3]<<std::endl;
					break;
				}
				case(2):
				{
					mData.z[counter[2]++]=atof(v[i].data());
					std::cout<<"z[0]:"<<mData.z[0]<<std::endl;
					std::cout<<"z[1]:"<<mData.z[1]<<std::endl;
					std::cout<<"z[2]:"<<mData.z[2]<<std::endl;
					std::cout<<"z[3]:"<<mData.z[3]<<std::endl;
					break;
				}
				case(3):
				{
					mData.w[counter[3]++]=atof(v[i].data());
					std::cout<<"w[0]:"<<mData.w[0]<<std::endl;
					std::cout<<"w[1]:"<<mData.w[1]<<std::endl;
					std::cout<<"w[2]:"<<mData.w[2]<<std::endl;
					std::cout<<"w[3]:"<<mData.w[3]<<std::endl;
					break;
				}
				default:
				{
					throw("an error occured");
					break;
				}
			}
		}
		memset(counter,0,10);//等于bzero(counter,10);
*/
		sleep(2);
		if(strncmp(buf, "exit", 4) == 0 || strcmp(buf, "") == 0)
		{
			//通知主线程。。。
			printf("GOODBYE\n");
			bzero(buf, 1024);
			pthread_cancel(send_thread);
			break;//退出
		}
	}
	
	pthread_exit(NULL);
}
std::vector<std::string> split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;

    str += pattern;//扩展字符串以方便操作
    int size = str.size();

    for (int i = 0; i<size; i++) {
        pos = str.find(pattern, i);
        if (pos<size) {
            std::string s = str.substr(i, pos - i);
            result.push_back(s);
            i = pos + pattern.size() - 1;
        }
    }
    return result;
}
