#ifndef SOCKET_CLIENT_H 
#define SOCKET_CLIENT_H
#include <string>
#include<sys/socket.h>
#include <netinet/in.h>

#include "dcm_data.h"

class SocketClient
{
    public:
        SocketClient();
        ~SocketClient();
        int socketConnect(std::string hostname,int port);
        static int hostname_to_ip(char const* hostname , char* ip);
        bool isConnected() const
        {
            return _isConnected;
        }
        void disconnect();
        int receive(char *data,size_t size);
        int sendToSoc(char *buf,size_t size);
        int sendCommand(command_t &com);
        
    private:
        struct sockaddr_in server;
        int _sock;
        bool _isConnected;
        
        
};

#endif
