/*! \file nao_walk_ros.h
 *
 *
 */

#ifndef nao_walk_ros_h
#define nao_walk_ros_h
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/WrenchStamped.h>
#include <eigen3/Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_stepplanner2D/RequestStepAction.h>
#include <humanoid_nav_msgs/ExecFootstepsAction.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <humanoid_nav_msgs/ExecFootstepsGoal.h>
#include <humanoid_nav_msgs/ExecFootstepsFeedback.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <simple_stepplanner2D/Step.h>
#include <simple_stepplanner2D/InitStep.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// NAOqi Headers
/*
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alcommon/almodulecore.h>
#include <alproxies/almemoryproxy.h>
#include <qi/os.hpp>
#include <alvalue/alvalue.h>
#include <alerror/alerror.h>
*/
// Boost Headers
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
#include <dynamic_reconfigure/server.h>
#include <nao_walk/GaitControlConfig.h>
#include <nao_walk/Crouch.h>
#include <nao_walk/WakeUp.h>
#include <nao_walk/sayText.h>

/**/
#include <nao_walk/DanceEvolution.h>
#include <nao_walk/EyeOfTheTiger.h>
#include <nao_walk/GangnamStyle.h>
#include <nao_walk/Macarena.h>
#include <nao_walk/TaiChi.h>
#include <nao_walk/Vangelis.h>
#include <nao_walk/WarmHello.h>
#include <nao_walk/Tired.h>
#include <nao_walk/BallTracker.h>

#include <nao_walk/socket_client.h>
#include "nao_walk/dcm_data.h"
#include <deque>
#include<map>
#include <nao_walk/behaviourAction.h>
#include <nao_walk/speekAction.h>

/**
 * \class nao_walk_ros
 *
 * \file nao_walk_ros.h
 **/

using namespace Eigen;


// namespace AL
// {
// class ALBroker;
// }


// // Helper definition
// template<typename T, size_t N>
// T * end(T (&ra)[N]) {
//     return ra + N;
// }
// 



// class nao_walk_ros : public AL::ALModule
class nao_walk_ros
{
    
    private:
        bool simpleStepAvailable;
        //actions
        typedef actionlib::ActionServer<humanoid_nav_msgs::ExecFootstepsAction>::GoalHandle FootstepsGoalH;
        actionlib::ActionServer<humanoid_nav_msgs::ExecFootstepsAction> *stepsActServer;
        FootstepsGoalH stepsGoal;
        bool hasStepGoal;
        
        typedef actionlib::ActionServer<nao_walk::behaviourAction>::GoalHandle BehaviourGoalH;
        actionlib::ActionServer<nao_walk::behaviourAction> *behaviourActServer;        
        BehaviourGoalH behaveGoal;
        bool hasBehaveGoal;
        
        typedef actionlib::ActionServer<nao_walk::speekAction>::GoalHandle SpeekGoalH;
        actionlib::ActionServer<nao_walk::speekAction> *speekActServer;        
        SpeekGoalH speekGoal;
//         bool hasSpeekGoal;
        
        //callbacks
        void footstepsExecutionCallback(FootstepsGoalH gh);
        void behaviourExecutionCallback(BehaviourGoalH gh);
        void speekExecutionCallback(SpeekGoalH gh);
        
        std::map<int,SpeekGoalH> speekMap;
        void clearCmd();
        
        
        std::string nao_hostname;
        int port;
        // ROS Standard Variables
//         ros::NodeHandle node_handle_;
        // Helper
        ros::Subscriber sub, footstep_sub, odomSub;
        ros::Publisher odom_pub, lfsr_pub, rfsr_pub, copr_pub, copl_pub, imu_pub, joint_state_pub;
        ros::ServiceServer serv_wakeUp, serv_crouch, serv_sayText;
        ros::ServiceServer serv_makarena, serv_danceEvolution, serv_gangnamStyle, serv_vangelis, serv_taichi, serv_eyeofthetiger, serv_warmhello, serv_tired, serv_balltracker;
        actionlib::SimpleActionClient<simple_stepplanner2D::RequestStepAction>* fc;
        simple_stepplanner2D::RequestStepGoal goal;
        ros::ServiceClient sc;
        boost::shared_ptr<nav_msgs::Odometry> odom_msg;
        geometry_msgs::WrenchStamped lfsr_msg, rfsr_msg;
        geometry_msgs::PointStamped copl_msg, copr_msg;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::JointState joint_state_msg;
        Vector3d copl, copr, RLegGRF, LLegGRF, RLegGRT, LLegGRT;
        int seq, fsr_seq;
        
        
        //!< Dynamic reconfigure server to allow config modifications at runtime
        boost::shared_ptr< dynamic_reconfigure::Server<nao_walk::GaitControlConfig> > dynamic_recfg_;
        void odomCallbackFromTopic(const nav_msgs::Odometry::ConstPtr &odom) ;
        void publishOdomToTf(const nav_msgs::Odometry::ConstPtr &odom );
        
        bool tf_from_kimenatics;
        SocketClient socketClient;
        
        unsigned int cmd_id;
        std::deque<command_t> cmd_q;
        command_t current_cmd;
        
        int sendData();
        inline int addCmd(command_t &cmd)
        {
            cmd_id++;
            cmd.id=cmd_id;
            cmd_q.push_back(cmd);
            return cmd_id;
        }
    public:

        nao_walk_ros();
        ~nao_walk_ros();
        void odomCallback();
            
        int readData(dcm_data_t &data);
        void generateOdomMsg(const dcm_data_t &dcm_data);
        void generateLFsrMsg(const dcm_data_t &dcm_data);
        void generateRFsrMsg(const dcm_data_t &dcm_data);
        void generateImuMsg(const dcm_data_t &dcm_data);
        void generateJointsMsg(const dcm_data_t &data);
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist_);
        
        
        bool sayTextCb(nao_walk::sayText::Request &req, nao_walk::sayText::Response &res);
        void  initialize(ros::NodeHandle nh);
        int   run();
        
        void activeCb();
        
        void  actionCb(const actionlib::SimpleClientGoalState& state,
        const simple_stepplanner2D::RequestStepResultConstPtr& result);
        void reconfigureCB(nao_walk::GaitControlConfig& config, uint32_t level);
        bool wakeUpCb(nao_walk::WakeUp::Request &req, nao_walk::WakeUp::Response &res);
        bool crouchCb(nao_walk::Crouch::Request &req, nao_walk::Crouch::Response &res);
};
#endif

