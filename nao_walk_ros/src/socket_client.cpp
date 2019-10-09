#include"nao_walk/socket_client.h"
#include<string.h> //memset
#include<errno.h> //For errno - the error number
#include<netdb.h>   //hostent
#include<arpa/inet.h>
#include <ros/ros.h>
#include <ros/console.h>
  #include <errno.h>
SocketClient::SocketClient()
    :_sock(-1),
    _isConnected(false)
{
}

SocketClient::~SocketClient()
{
    if(_isConnected)
        disconnect();
}

void SocketClient::disconnect()
{
    _isConnected=false;
    close(_sock);    
    _sock=-1;
}

int SocketClient::socketConnect(std::string hostname, int port)
{
//     int sock;
//     _sock
//     char message[1000] , server_reply[2000];
    
    //Create socket
    _sock = socket(AF_INET , SOCK_STREAM , 0);
    if (_sock == -1)
    {
        ROS_ERROR("Could not create socket");
//         perror("Could not create socket");
        return 1;
    }
//     ROS_INFO("Socket created");
    
    char ip[64];
    hostname_to_ip(hostname.c_str(),ip);
    
    server.sin_addr.s_addr = inet_addr(ip);
    server.sin_family = AF_INET;
    server.sin_port = htons( port );

    //Connect to remote server
//     ROS_INFO("Connecting to %s:%d",ip,port);
    if (connect(_sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
//         ROS_ERROR("connect failed. Error");
        close(_sock);
        return 1;
    }
    _isConnected=true;
//     ROS_INFO("Connected to socket");
    return 0;
}

int SocketClient::hostname_to_ip(char const*hostname , char* ip)
{
    struct hostent *he;
    struct in_addr **addr_list;
    int i;
        
    if ( (he = gethostbyname( hostname ) ) == NULL) 
    {
        // get the host info
//         herror("gethostbyname");
        return 1;
    }

    addr_list = (struct in_addr **) he->h_addr_list;
    
    for(i = 0; addr_list[i] != NULL; i++) 
    {
        //Return the first one;
        strcpy(ip , inet_ntoa(*addr_list[i]) );
        return 0;
    }
    
    return 1;
}

int SocketClient::receive(char *data,size_t size)
{
    //ROS_INFO("SIZE %lu",size);
    size_t red=0;
    int r;
    while(red<size)
    {
        r=recv(_sock,data+red,size-red,0);
        if(r<0)
        {
//             ROS_ERROR("Error reading from socket");
            disconnect();
            return -1;
        }
        else if(r==0)
        {
//             ROS_ERROR("Socket closed remotely.");
            disconnect();
            return -1;
        }
        red+=r;
    }
    //ROS_INFO("Read %lu",red);
    return 0;
}

int SocketClient::sendToSoc(char *buf,size_t size)
{
    size_t written=0;
    int r=0;
    while(written<size)
    {
        r=send(_sock,buf+written,size-written,0);
        if(r<0)
        {
//             ROS_ERROR("Error writting to socket!");
            disconnect();
            return -1;
        }
        else if(r==0)
        {
//             ROS_ERROR("Socket closed remotely.");
            disconnect();
            return -1;
        }
        written+=r;
    }
    //ROS_INFO("Written %lu",written);
    return 0;
}

int SocketClient::sendCommand(command_t &com)
{
    int data[4];
    data[0]=WALK_ENGINE_ID;
    data[1]=com.command;
    data[2]=com.id;
    data[3]=com.data_size;
    int ret=sendToSoc((char*)data,4*sizeof(int) );
    if(ret<0)
        return ret;
    if(com.data_size>0)
        return sendToSoc((char*)com.data,com.data_size);
}


