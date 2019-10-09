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
 
 
 
  // Create your own broker
//     boost::shared_ptr<AL::ALBroker> broker;
//     try
//     {
// //         broker = AL::ALBroker::createBroker(broker_name,broker_ip,broker_port,pip,pport,0);
//     }
//     catch(...)
//     {
//         ROS_ERROR("Failed to connect to Broker at %s:%d!",pip.c_str(),pport);
// //         return -1;
//     }

    // Deal with ALBrokerManager singleton (add your broker into NAOqi)
//     AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
//     AL::ALBrokerManager::getInstance()->addBroker(broker);
     
//     boost::shared_ptr<nao_walk_ros> nao_walk = AL::ALModule::createModule<nao_walk_ros>(broker, "nao_walk_ros");
   
//     if(broker->isModulePresent("LowLevelPlanner"))
//           ROS_INFO("Nao Walk Engine Module is online!");
//     else
//     {
//           ROS_ERROR("Nao Walk Engine Module is offline!");
//           return -1;
//     }
// 
//     boost::shared_ptr<AL::ALProxy> nao_walk_engine = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(broker, "LowLevelPlanner"));
//     nao_walk->setEngineProxy(nao_walk_engine);
// 
//    
//     nao_walk->initialize(nh);
    // if(!broker->isModulePresent("LowLevelPlanner"))
    // {
    //      ROS_ERROR("Could not connect to the Nao robot!");
    //      AL::ALBrokerManager::getInstance()->killAllBroker();
    //      AL::ALBrokerManager::kill();
    //      return -1;
    // }
//     boost::shared_ptr<AL::ALBroker> broker;
//     broker = AL::ALBroker::createBroker(broker_name,broker_ip,broker_port,pip,pport,0);
    
//     AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
//      AL::ALBrokerManager::getInstance()->addBroker(broker);
// boost::shared_ptr<nao_walk_ros> nao_walk = AL::ALModule::createModule<nao_walk_ros>(broker, "nao_walk_ros");

    nao_walk_ros *nao_walk=new nao_walk_ros();
    nao_walk->initialize(nh);
    // Run Nao Driver Loop
    nao_walk->run();
    //Done here
//     AL::ALBrokerManager::getInstance()->killAllBroker();
//     AL::ALBrokerManager::kill();
    ROS_INFO( "Quitting... " );

}