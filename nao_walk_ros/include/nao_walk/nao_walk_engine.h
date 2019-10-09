#ifndef NAO_WALK_ENGINE_H
#define NAO_WALK_ENGINE_H

#include "socket_client.h"
#include "data_msg.h"

class NaoWalkEngine
{
    public:
        NaoWalkEngine(std::string ip,int port);
        
        int readOdom(odom_t *odom);

    private:
};
#endif
