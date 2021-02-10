#include "nao_walk/nao_walk_ros.h"
#include "nao_walk/joint_states_table.h"
#include <vector>

#include <string.h>
#include <boost/concept_check.hpp>





nao_walk_ros::nao_walk_ros()
{    
    hasStepGoal=false;
    hasBehaveGoal=false;
//     socketClient.socketConnect("Odysseus.ocal",8080);
}

void nao_walk_ros::clearCmd()
{
    if(hasStepGoal)
    {
        stepsGoal.setAborted();
        hasStepGoal=false;
    }
    
    if(hasBehaveGoal)
    {
        behaveGoal.setAborted();
        hasBehaveGoal=false;
    }
    
    cmd_q.clear();
    current_cmd.command=NONE;
    current_cmd.id=NONE;
    if(current_cmd.data!=0)
    {
        free(current_cmd.data);
        current_cmd.data=0;
    }
    
}

nao_walk_ros::~nao_walk_ros()
{
    if(socketClient.isConnected())
        socketClient.disconnect();
}

void nao_walk_ros::generateLFsrMsg(const dcm_data_t &dcm_data) 
{
    lfsr_msg.header.seq = fsr_seq;
    lfsr_msg.header.stamp = ros::Time::now();
    lfsr_msg.header.frame_id = "l_sole";
    
    copl = Vector3d(dcm_data.copl[0],dcm_data.copl[1],dcm_data.copl[2]);
    
    LLegGRF = Vector3d(0,0,dcm_data.copl[3]);
    LLegGRT = copl.cross(LLegGRF);
    
    lfsr_msg.wrench.force.x = LLegGRF(0);
    lfsr_msg.wrench.force.y = LLegGRF(1);
    lfsr_msg.wrench.force.z = LLegGRF(2);

    lfsr_msg.wrench.torque.x = LLegGRT(0);
    lfsr_msg.wrench.torque.y = LLegGRT(1);
    lfsr_msg.wrench.torque.z = LLegGRT(2);
    
    copl_msg.header.seq = fsr_seq;
    copl_msg.header.stamp = ros::Time::now();
    copl_msg.header.frame_id = "l_sole";
    
    copl_msg.point.x = copl(0);
    copl_msg.point.y = copl(1);
    copl_msg.point.z = copl(2);
    /*
    copr_msg.header.seq = fsr_seq;
    copr_msg.header.stamp = ros::Time::now();
    copr_msg.header.frame_id = "r_sole";
    
    copr_msg.point.x = copr(0);
    copr_msg.point.y = copr(1);
    copr_msg.point.z = copr(2);
    */
}

void nao_walk_ros::generateRFsrMsg(const dcm_data_t &dcm_data) 
{
    rfsr_msg.header.seq = fsr_seq;
    rfsr_msg.header.stamp = ros::Time::now();
    rfsr_msg.header.frame_id = "r_sole";
    
    copr = Vector3d(dcm_data.copr[0],dcm_data.copr[1],dcm_data.copr[2]);
    RLegGRF = Vector3d(0,0,dcm_data.copr[3]);
    RLegGRT = copr.cross(RLegGRF);
    
    rfsr_msg.wrench.force.x = RLegGRF(0);
    rfsr_msg.wrench.force.y = RLegGRF(1);
    rfsr_msg.wrench.force.z = RLegGRF(2);

    rfsr_msg.wrench.torque.x = RLegGRT(0);
    rfsr_msg.wrench.torque.y = RLegGRT(1);
    rfsr_msg.wrench.torque.z = RLegGRT(2);
    
    copr_msg.header.seq = fsr_seq;
    copr_msg.header.stamp = ros::Time::now();
    copr_msg.header.frame_id = "r_sole";
    
    copr_msg.point.x = copr(0);
    copr_msg.point.y = copr(1);
    copr_msg.point.z = copr(2);
}

void nao_walk_ros::generateImuMsg(const dcm_data_t &dcm_data) 
{
    
    imu_msg.header.frame_id = "base_link";
    imu_msg.header.seq = fsr_seq;
    imu_msg.header.stamp = ros::Time::now();
    
    imu_msg.linear_acceleration.x = dcm_data.acc[0];
    imu_msg.linear_acceleration.y = dcm_data.acc[1];
    imu_msg.linear_acceleration.z = dcm_data.acc[2];

    imu_msg.angular_velocity.x = dcm_data.gyro[0];
    imu_msg.angular_velocity.y = dcm_data.gyro[1];
    imu_msg.angular_velocity.z = dcm_data.gyro[2];
}

void nao_walk_ros::generateJointsMsg(const dcm_data_t &data)
{
    joint_state_msg.header.seq = fsr_seq;
    joint_state_msg.header.stamp = ros::Time::now();

    for(int i=0;i<joint_state_names.size();i++)
    {
        joint_state_msg.position[i]=data.joint_states[i];
    }   
}

void nao_walk_ros::odomCallbackFromTopic(const nav_msgs::Odometry::ConstPtr &odom) {
    publishOdomToTf(odom);
}

void nao_walk_ros::publishOdomToTf(const nav_msgs::Odometry::ConstPtr &odom)
{                  
    tf::Transform transform;
    transform.setOrigin(  tf::Vector3(
        odom->pose.pose.position.x,  //x
        odom->pose.pose.position.y,  //y
        odom->pose.pose.position.z  //z
    ) );                
    tf::Quaternion q(
        odom->pose.pose.orientation.x,  //w
        odom->pose.pose.orientation.y,  //y
        odom->pose.pose.orientation.z,  //z
        odom->pose.pose.orientation.w  //w
    );
    transform.setRotation(q);
    static  tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, odom->header.stamp, "odom", "base_link"));
}

int nao_walk_ros::readData(dcm_data_t &data)
{
    if(socketClient.receive((char*)&data,sizeof(data) )<0 )
    {
        clearCmd();
        return -1;
    }

    generateImuMsg(data);
    generateLFsrMsg(data);
    generateRFsrMsg(data);
    generateJointsMsg(data);
    
    imu_pub.publish(imu_msg);
    lfsr_pub.publish(lfsr_msg);
    copl_pub.publish(copl_msg);
    copr_pub.publish(copr_msg);
    rfsr_pub.publish(rfsr_msg);
    joint_state_pub.publish(joint_state_msg);

    if(tf_from_kimenatics )
    {
//         boost::shared_ptr<nav_msgs::Odometry> odom_msg(new  nav_msgs::Odometry); 
        generateOdomMsg(data);
        odom_pub.publish(odom_msg);
        publishOdomToTf(odom_msg); 
    }
    
    fsr_seq++;
    seq++;
        
//     ROS_INFO("VALID:%d",data.isValid);
//      ROS_INFO("COM:%d,%d,%d",data.last_cmd,data.last_cmd_id,data.last_cmd_status);
    if(data.last_cmd==NONE)
        return 0;


    if( data.last_cmd_id==current_cmd.id &&
        data.last_cmd_status!=STATUS_PENDING)     //COMMAND IS FINISHED
    {
        current_cmd.command=NONE;
        current_cmd.id=NONE;
        if(current_cmd.data!=0)
        {
            free(current_cmd.data);
            current_cmd.data=0;
        }
        
        if(data.last_cmd==STEPS && hasStepGoal)
        {            
            hasStepGoal=false;
            if(data.last_cmd_status==STATUS_DONE)
            {
                ROS_INFO("Steps finished");
                stepsGoal.setSucceeded();
            }
            else
            {
                ROS_INFO("Steps aborted");
                stepsGoal.setAborted();
            }
        }
        else if(data.last_cmd==SPEEK)
        {
            int id=data.last_cmd_id;
            auto it=speekMap.find(id);
            if(it==speekMap.end())
            {
                ROS_ERROR("Speek command with id:%d does not exist.",id);
                return 0;
            }
            SpeekGoalH sg=it->second;
            if(data.last_cmd_status==STATUS_DONE)
                sg.setSucceeded();
            else
                sg.setAborted();
            speekMap.erase (it);

        }
        else if(data.last_cmd>=COMMANDS_SIZE && data.last_cmd<BEHAVIOURS_SIZE && hasBehaveGoal)
        {
            ROS_INFO("GET_BEH");
            hasBehaveGoal=false;
            if(data.last_cmd_status==STATUS_DONE)
                behaveGoal.setSucceeded();
            else
                behaveGoal.setAborted();
        }  
    }
    return 0; 
}

int nao_walk_ros::sendData()
{
//     ROS_INFO("last cmd:%d",current_cmd.command);
    command_t cmd;
    if(current_cmd.id==NONE && !cmd_q.empty())
    {
        cmd=cmd_q.front();
        cmd_q.pop_front();
        current_cmd=cmd;
//         ROS_INFO("new cmd:%d",current_cmd.command);
    }
    else
    {
        cmd.command=NONE;
        cmd.id=NONE;
        cmd.data_size=0;
        cmd.data=0;
    }

    if(socketClient.sendCommand( cmd )<0)
    {
        clearCmd();
        return -1;
    }
    return 0;
}

void nao_walk_ros::generateOdomMsg(const dcm_data_t &dcm_data)
{
//     boost::shared_ptr<nav_msgs::Odometry> odom_msg(new  nav_msgs::Odometry); 
    odom_msg->header.seq=seq;
    odom_msg->header.stamp=ros::Time::now();
    odom_msg->header.frame_id="odom";
    odom_msg->child_frame_id="base_link";
    odom_msg->pose.pose.position.x = dcm_data.odom.trans[0];
    odom_msg->pose.pose.position.y = dcm_data.odom.trans[1];
    odom_msg->pose.pose.position.z = dcm_data.odom.trans[2];
    odom_msg->pose.pose.orientation.x = dcm_data.odom.quat[0];
    odom_msg->pose.pose.orientation.y = dcm_data.odom.quat[1];
    odom_msg->pose.pose.orientation.z = dcm_data.odom.quat[2];
    odom_msg->pose.pose.orientation.w = dcm_data.odom.quat[3];
    for(int i=0;i<36;i++)
        odom_msg->pose.covariance[i] = 0;
}

void nao_walk_ros::speekExecutionCallback(SpeekGoalH gh)
{    
    ROS_INFO("A speak message %s",gh.getGoal()->text.c_str());
    command_t cmd;
    cmd.command=SPEEK;
    cmd.data_size=gh.getGoal()->text.size()+1;
    cmd.data=malloc(cmd.data_size+1);
    memcpy(cmd.data,gh.getGoal()->text.c_str(),cmd.data_size);
    char *char_data=(char*)cmd.data;
    char_data[gh.getGoal()->text.size()]='\0';
    addCmd(cmd);
    
    speekMap[cmd.id]=gh;
    gh.setAccepted();
}

void nao_walk_ros::behaviourExecutionCallback(BehaviourGoalH gh)
{
    ROS_INFO("Let's dance");
    if(hasBehaveGoal)
        gh.setAborted();
    
    int beh_id=gh.getGoal()->behaviour_id+COMMANDS_SIZE;
    
    command_t cmd;
    cmd.command=beh_id;
    cmd.data_size=0;
    cmd.data=0;   
    addCmd(cmd);
    
    hasBehaveGoal=true;
    behaveGoal=gh;
    gh.setAccepted();
}

void nao_walk_ros::footstepsExecutionCallback(FootstepsGoalH gh)
{
    if(hasStepGoal)
        gh.setAborted();

    step_t *steps=(step_t *)malloc(sizeof(step_t)*gh.getGoal()->footsteps.size());
    humanoid_nav_msgs::ExecFootstepsGoalConstPtr goal=gh.getGoal();
    stepsGoal=gh;
    
    for(int i=0;i<goal->footsteps.size();i++)
    {
        if(goal->footsteps[i].leg==1)
            steps[i].leg=LEFT;
        else
            steps[i].leg=RIGHT;

        steps[i].x=goal->footsteps[i].pose.x;
        steps[i].y=goal->footsteps[i].pose.y;
        steps[i].theta=goal->footsteps[i].pose.theta;
        
        //if(i==goal->footsteps.size()-1)
        //     steps[i].cmd = STAND;
        //else
            steps[i].cmd = WALK;
    }
    command_t cmd;
    cmd.command=STEPS;
    cmd.data_size=sizeof(step_t)*goal->footsteps.size();
    cmd.data=(void*)steps;
    addCmd(cmd);
    
    hasStepGoal=true;
    gh.setAccepted();
}

void nao_walk_ros::reconfigureCB(nao_walk::GaitControlConfig& config, uint32_t level)
{      
    command_t cmd;
    cmd.command=RECONFIG;
    cmd.data_size=sizeof(float)*RECONFIG_SIZE;
    cmd.data=(void*)malloc(cmd.data_size);
    
    float *data=(float*)cmd.data;
    data[0] = config.Observer_COPX;
    data[1] = config.Observer_COPY;
    data[2] = config.Observer_CoMX;
    data[3] = config.Observer_CoMY;
    data[4] = config.COP_NoiseX;
    data[5] = config.COP_NoiseY;
    data[6] = config.CoM_NoiseX;
    data[7] = config.CoM_NoiseY;
    data[8] = config.Kp_PitchT;
    data[9] = config.Kd_PitchT;
    data[10] = config.Kp_RollT;
    data[11] = config.Kd_RollT;
    data[12] = config.kfx;
    data[13] = config.kfy;
    data[14] = config.StepHeight;

    addCmd(cmd);
        ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", 
            config.Observer_COPX, config.Observer_COPY, config.Observer_CoMX,config.Observer_CoMY, config.COP_NoiseX, config.COP_NoiseY, config.CoM_NoiseX, config.CoM_NoiseY,
            config.Kp_PitchT, config.Kd_PitchT, config.Kp_RollT, config.Kd_RollT,  config.kfx,  config.kfy, config.StepHeight);
    /*
    if(proxy_ready_)
    {
  

      AL::ALValue data;
      data.arraySetSize(15);
      data[0] = config.COP_Noise;
      data[1] = config.CoM_Noise;
      data[2] = config.Observer_COP;
      data[3] = config.Observer_CoM;
      data[4] = config.Kp_Pitch;
      data[5] = config.Kd_Pitch;
      data[6] = config.Kp_Roll;
      data[7] = config.Kd_Roll;
      data[8] = config.amX;
      data[9] = config.amY;
      data[10] = config.StepHeight;
      data[11] = config.kfx;
      data[12] = config.kfy;
      data[13] = config.kcx;
      data[14] = config.kcy;
//       nao_walk_engine->callVoid("setRobotConfig",data);
    }
    */
}

void nao_walk_ros::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist_)
{
    command_t cmd;
    cmd.command=VELOCITY;
    cmd.data_size=sizeof(velocity_t);
    cmd.data=(void*)malloc(cmd.data_size);

    velocity_t *vel=(velocity_t*)cmd.data;

    vel->vx=twist_->linear.x;
    vel->vy=twist_->linear.y;
    vel->vw=twist_->angular.z;

    addCmd(cmd);
}


bool nao_walk_ros::wakeUpCb(nao_walk::WakeUp::Request &req, nao_walk::WakeUp::Response &res)
{
    command_t cmd;
    cmd.command=WAKE_UP;    
    cmd.data=0;
    cmd.data_size=0;
    addCmd(cmd);
    return true;
}

bool nao_walk_ros::crouchCb(nao_walk::Crouch::Request &req, nao_walk::Crouch::Response &res)
{
    ROS_INFO("Received crouch command.");
    command_t cmd;
    cmd.command=CROUCH;    
    cmd.data=0;
    cmd.data_size=0;
    addCmd(cmd);
    return true;
}

void nao_walk_ros::initialize(ros::NodeHandle nh)
{
    cmd_id=0;
    current_cmd.id=NONE;
    current_cmd.command=NONE;
    current_cmd.data=0;
    current_cmd.data_size=0;
	 // Initialize ROS nodes
//     nh _= nh;
    
    
    tf_from_kimenatics=true;
    std::string odom_topic;
    if( nh.getParam("odom_topic", odom_topic) ) 
    {
        tf_from_kimenatics=false;
        ROS_INFO("Reading odometry from topic: %s ", odom_topic.c_str() );
    }
    else 
    {
        ROS_INFO("Reading odometry from kimenatics");
        odom_msg=boost::shared_ptr<nav_msgs::Odometry>(new  nav_msgs::Odometry); 
    }
    
    if(!nh.getParam("nao_hostname", nao_hostname) ) 
    {
        nao_hostname=std::string("Odysseus.local");        
    }
    if(!nh.getParam("port", port) ) 
    {
        port=8080;
    }
//     ROS_INFO("Nao hostname %s",nao_hostname.c_str());
    
    
    joint_state_msg.name.reserve(joint_state_names.size());
    joint_state_msg.position.reserve(joint_state_names.size());
    joint_state_msg.velocity.reserve(joint_state_names.size());
    joint_state_msg.effort.reserve(joint_state_names.size());
    
    for(int i=0;i<joint_state_names.size();i++)
    {
        joint_state_msg.name.push_back(joint_state_names[i]);
        joint_state_msg.position.push_back(0.0); //to be replaced later with actual data 
    }
   
    seq=0;
    fsr_seq=0;

    odom_pub = nh.advertise<nav_msgs::Odometry>("/nao_robot/odom",2);
    lfsr_pub = nh.advertise<geometry_msgs::WrenchStamped>("/nao_robot/LLeg/force_torque_states",1000);
    copl_pub = nh.advertise<geometry_msgs::PointStamped>("/nao_robot/LLeg/COP",1000);
  

    rfsr_pub = nh.advertise<geometry_msgs::WrenchStamped>("/nao_robot/RLeg/force_torque_states",1000);
    copr_pub = nh.advertise<geometry_msgs::PointStamped>("/nao_robot/RLeg/COP",1000);

    imu_pub =  nh.advertise<sensor_msgs::Imu>("/nao_robot/imu",1000);
    joint_state_pub =  nh.advertise<sensor_msgs::JointState>("/joint_states",10);

    sub = nh.subscribe("/nao_motion/cmd_vel", 1,&nao_walk_ros::cmdVelCallback,this);
        
    stepsActServer=new actionlib::ActionServer<humanoid_nav_msgs::ExecFootstepsAction>(nh,
        "/footsteps_execution",boost::bind(&nao_walk_ros::footstepsExecutionCallback,this,_1),false);
    stepsActServer->start();
    
    behaviourActServer=new actionlib::ActionServer<nao_walk::behaviourAction>(nh,
        "/behaviour_execution",boost::bind(&nao_walk_ros::behaviourExecutionCallback,this,_1),false);      
    behaviourActServer->start();
    
    speekActServer=new actionlib::ActionServer<nao_walk::speekAction>(nh,
        "/speed_execution",boost::bind(&nao_walk_ros::speekExecutionCallback,this,_1),false);      
    speekActServer->start();    




    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<nao_walk::GaitControlConfig> >(nh);
    dynamic_reconfigure::Server<nao_walk::GaitControlConfig>::CallbackType cb = boost::bind(&nao_walk_ros::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

    ROS_INFO("Nao Walk Engine ROS Wrapper Initialized");
}

int nao_walk_ros::run()
{  
    static ros::Rate rate(100);
    while(ros::ok())
    {
        if(socketClient.isConnected())
        {
            dcm_data_t data;
            if(readData(data)==0 )
                sendData();
        }
        else
        {
//             ROS_INFO("hostname:%s",nao_hostname.c_str() );
            socketClient.socketConnect(nao_hostname,port);
        }
        ros::spinOnce();
        rate.sleep();
    }

    if(socketClient.isConnected())
        socketClient.disconnect();
}

