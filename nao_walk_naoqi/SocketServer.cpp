#include"SocketServer.h" 


#include <sys/socket.h> 
#include <stdlib.h> 
#include <stdio.h>
#include"Logs.h"

#include <sys/types.h>
#include <sys/socket.h>

SocketServer::SocketServer()
    :server_fd(-1),
    client_conn(-1)    
{}

SocketServer::~SocketServer()
{
    socClose();
}

int SocketServer::socInit(int port)
{
    int opt = 1;     
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        LOG_ERR("socket failed"); 
        return -1;
    }
    /*
     * No SO_REUSEPORT
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) 
    */  
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)))        
    { 
        LOG_ERR("setsockopt"); 
        return -1;
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( port ); 

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) 
    { 
        LOG_ERR("bind failed"); 
        return -1;
    }  
    return 0;
}

int SocketServer::socListen()
{
    if (listen(server_fd, 1) < 0) 
    { 
        LOG_ERR("listen"); 
        return -1; 
    } 
    return 0;
}

int SocketServer::socAccept()
{
    int addrlen = sizeof(address); 
    if ((client_conn = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        client_conn=-1;
        LOG_ERR("Socket accepting error"); 
        return -1;
    } 
    LOG_INF("Socket accepted");
    return 0;
}

int SocketServer::sendToSoc(char *buf,size_t size)
{
    size_t written=0;
    int r=0;
    while(written<size)
    {
        r=send(client_conn,buf+written,size-written,0);
        if(r<0)
        {
            LOG_ERR("Error writting to socket!");
            close(client_conn);
            client_conn=-1;
            return -1;
        }
        if(r==0) //soc closed
        {
            LOG_ERR("Socket closed.");
            close(client_conn); 
            client_conn=-1;
            return -1;
        }
        written+=r;
    }
    //ROS_INFO("Written %lu",written);
    return 0;
}

int SocketServer::receive(char *data,size_t size)
{
    //ROS_INFO("SIZE %lu",size);
    size_t red=0;
    int r;
    while(red<size)
    {
//         r=read(client_conn,data+red,size-red);
        //TODO set timeout
        r=recv(client_conn,data+red,size-red,0);
        if(r<0)
        {
            LOG_ERR("Error reading from socket");
            close(client_conn);
            client_conn=-1;
            return -1;
        }
        if(r==0) //soc closed
        {
            LOG_ERR("Socket closed.");
            close(client_conn); 
            client_conn=-1;
            return -1;
        }
        red+=r;
    }
    //ROS_INFO("Read %lu",red);
    return 0;
}

int SocketServer::receiveCommand(command_t &com)
{
    int data[4];
//     data[0]=WALK_ENGINE_ID;
//     data[1]=com.command;
//     data[2]=com.id;
//     data[3]=com.data_size;
//     int ret=send(data,4*sizeof(int) );
    
    int ret=receive((char*)data,4*sizeof(int) );    
    if(ret<0)
        return ret;
    if(data[0]!=WALK_ENGINE_ID)
    {
        LOG_ERR("Unkwown data format"); 
        return -1;
    }
    com.command=data[1];
    com.id=data[2];
    com.data_size=data[3];
    
    if(com.data_size>0)
    {
        com.data=malloc(com.data_size);
        return receive((char*)com.data,com.data_size);
    }
    com.data=0;
    return 0;
}


void SocketServer::socClose()
{
    if(client_conn!=-1)
    {
        close(client_conn);
        client_conn=-1;
    }
    
    if(server_fd>0)
        close(server_fd);
}