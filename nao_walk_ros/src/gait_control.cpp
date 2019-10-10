#include <iostream>
#include "nao_walk/nao_walk_ros.h"
using std::string;
using std::cerr;
using std::endl;

int main( int argc, char** argv )
{
    int pport;
    string pip;
    // A broker needs a name, an IP and a port:
    std::string broker_name = "Nao Walk ROS Wrapper Broker";    // FIXME: would be a good idea to look for a free port first
    int broker_port;
    // listen port of the broker (here an anything)
    string broker_ip;
    ros::init(argc, argv, "nao_walk");
    ros::NodeHandle nh("~"); //For parameter server
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }

  // Load Params from Parameter Server
    nh.getParam("RobotIP", pip);
    nh.getParam("RobotPort", pport);
    nh.getParam("DriverBrokerPort", broker_port);
    nh.getParam("DriverBrokerIP", broker_ip);
 

    nao_walk_ros *nao_walk=new nao_walk_ros();
    nao_walk->initialize(nh);
    // Run Nao Driver Loop
    nao_walk->run();
    //Done here
//     AL::ALBrokerManager::getInstance()->killAllBroker();
//     AL::ALBrokerManager::kill();
    ROS_INFO( "Quitting... " );

}