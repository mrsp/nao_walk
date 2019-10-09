#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H
#include <string>
#include <boost/concept_check.hpp>
#include <netinet/in.h> 
#include "DcmData.h"

class SocketServer
{
    public:
        SocketServer();
        ~SocketServer();
        int socInit(int port);
        int socListen();
        int socAccept();
        bool hasClient() const
        {
            return client_conn>0;
        }
        
        void socClose();
        int sendToSoc(char *buf,size_t size);
        int receive(char *buf,size_t size);
        int receiveCommand(command_t &com);
    private:
        int server_fd;
        int client_conn;        
        struct sockaddr_in address; 
};
#endif