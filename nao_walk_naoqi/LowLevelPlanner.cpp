#include "LowLevelPlanner.h"
#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <qi/log.hpp>
#include "Logs.h"
#include "behaviourMap.h"
// #include <pthread.h>

#include"Logs.h"


const float standingStiffness=0.9f;
// const float LOOP_TIME = 200000.0f;
// const float LOOP_TIME = 10000;
const float LOOP_TIME = 5000;


LowLevelPlanner::LowLevelPlanner(boost::shared_ptr<AL::ALBroker> broker, const std::string &name)
    :AL::ALModule(broker,name),
    last_dcm_data_stamp(0),
    curr_dcm_data(dcm_data),
    last_cmd(NONE),
    last_cmd_id(INV),
    last_cmd_param(INV),
    last_cmd_status(INV),
    first_step_id(0),
    last_step_id(INV)
{    
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);    
    setModuleDescription("Nao Walk Engine Module");
    dcm_data[0].stamp=0;
    dcm_data[1].stamp=1;
    
    dcm_data[0].dcm_data.isValid=false;
    dcm_data[1].dcm_data.isValid=false;
    /*
    functionName("requestStepCallback", getName(), "");
    BIND_METHOD(LowLevelPlanner::requestStepCallback);
    
//     functionName("stepCallback", getName(), "");
//     BIND_METHOD(LowLevelPlanner::stepCallback);
    
    functionName("stop", getName(), "");
    BIND_METHOD(LowLevelPlanner::stop);
    
    
    functionName("start", getName(), "");
    BIND_METHOD(LowLevelPlanner::start);
    
    functionName("getFootSteps", getName(), "");
    BIND_METHOD(LowLevelPlanner::getFootSteps);
    
    curr_dcm_data
    functionName("isFootStepExecuted", getName(), "");
    BIND_METHOD(LowLevelPlanner::isFootStepExecuted);
    
    functionName("setFootSteps", getName(), "");
    BIND_METHOD(LowLevelPlanner::setFootSteps);
    
    functionName("setDCMState", getName(), "");
    BIND_METHOD(LowLevelPlanner::setDCMState);
    
    
    functionName("getDCMState", getName(), "");
    BIND_METHOD(LowLevelPlanner::getDCMState);
    
    functionName("setRobotConfig", getName(), "");
    BIND_METHOD(LowLevelPlanner::setRobotConfig);
    
    functionName("runBehavior", getName(), "");
    BIND_METHOD(LowLevelPlanner::runBehavior);
    
    functionName("sayText", getName(), "");
    BIND_METHOD(LowLevelPlanner::sayText);
    */
    sockServer.socInit(8080);
    sockServer.socListen();
    
}


void LowLevelPlanner::generateDebugDataEvent()
{
    if (engine != NULL && engine->isEngineInitialized){
        debug_data[0] = engine->tp.ZMPbuffer[0](0);
        debug_data[1] = engine->tp.ZMPbuffer[0](1);
        debug_data[2] = engine->tp.ZMPbuffer[0](2);
        
        debug_data[3] = engine->NaoLIPM.comx_d;
        debug_data[4] = engine->NaoLIPM.comy_d;
        debug_data[5] = engine->NaoLIPM.vrpx_d;
        debug_data[6] = engine->NaoLIPM.vrpy_d;
        
        debug_data[7] = engine->copi(0);
        debug_data[8] = engine->copi(1);
    }
    fMemoryProxy->raiseEvent("debug", debug_data);
    
}

void LowLevelPlanner::generateDcmDataEvent()
{
    dcm_data_stamp_t *new_dcm_data_stamp;    

    if(curr_dcm_data==dcm_data)
        new_dcm_data_stamp=dcm_data+1;
    else
        new_dcm_data_stamp=dcm_data;

    dcm_data_t *new_dcm_data=&(new_dcm_data_stamp->dcm_data);
    
    if( (dcm_state == DCM_STOP ||dcm_state == DCM_READY_TO_STOP) || !engine->isEngineInitialized  )
    {
        new_dcm_data->isValid=false;
        new_dcm_data->odom=curr_dcm_data->dcm_data.odom;
        new_dcm_data->last_footstep=curr_dcm_data->dcm_data.last_footstep;
        new_dcm_data->last_footstep=INV;
    }
    else
    {
        new_dcm_data->isValid=true;
        //Odometry
        new_dcm_data->odom.trans[0]=engine->odomTrans(0);
        new_dcm_data->odom.trans[1]=engine->odomTrans(1);
        new_dcm_data->odom.trans[2]=engine->odomTrans(2);
        
        new_dcm_data->odom.quat[0]=engine->odomQuat.x();
        new_dcm_data->odom.quat[1]=engine->odomQuat.y();
        new_dcm_data->odom.quat[2]=engine->odomQuat.z();
        new_dcm_data->odom.quat[3]=engine->odomQuat.w();
        
        //footsteps
        new_dcm_data->last_footstep=engine->last_step_id;
        
    }
    //     std::cout<<"odom"<<std::endl;
    //     new_dcm_data->odom.double_support=engine->double_support;
    //     std::cout<<"odom generated"<<std::endl;
        //copl
    for(int i=0;i<3;i++)
        new_dcm_data->copl[i]=copl(i);
    new_dcm_data->copl[3]=weightl * 9.80665;
//     std::cout<<"TEST2"<<std::endl;
//     std::cout<<"copl generated"<<std::endl;
    //copr
    for(int i=0;i<3;i++)
        new_dcm_data->copr[i]=copr(i);
    new_dcm_data->copr[3]=weightr * 9.80665;
    
//     std::cout<<"copr generated"<<std::endl;
    //Acc
    for(int i=0;i<3;i++)
        new_dcm_data->acc[i]=Acc(i);
//     std::cout<<"Acc generated"<<std::endl;
    //Gyro
    for(int i=0;i<3;i++)
        new_dcm_data->gyro[i]=Gyro(i);
//     std::cout<<"Gyro generated"<<std::endl;
    //joint_states
    for(int i=0;i<26;i++)
        new_dcm_data->joint_states[i]=joint_states[i];
//     std::cout<<"joint_states generated"<<std::endl;

//     new_dcm_data->last_footstep=engine->last_step_id;
    //stamp
    new_dcm_data_stamp->stamp=curr_dcm_data->stamp+1;
    
    //Do change the Pointer
    curr_dcm_data=new_dcm_data_stamp;
    pthread_cond_signal(&cond);
}

/*
void LowLevelPlanner::generateOdometryEvent() {
    if (engine != NULL && engine->isEngineInitialized){
        odom[0]= (float) engine->odomTrans(0);
        odom[1]= (float) engine->odomTrans(1);
        odom[2]= (float) engine->odomTrans(2);
        
        odom[3]= (float) engine->odomQuat.x();
        odom[4]= (float) engine->odomQuat.y();
        odom[5]= (float) engine->odomQuat.z();
        odom[6]= (float) engine->odomQuat.w();
        odom[7]= (float)  engine->double_support_id;
    }
    fMemoryProxy->raiseEvent("odometry", odom);
    
}


void LowLevelPlanner::generateSensorDataEvent() {
    
    data_[0]=copl(0);
    data_[1]=copl(1);
    data_[2]=copl(2);
    data_[3]=weightl * 9.80665;
    
    data_[4]=copr(0);
    data_[5]=copr(1);
    data_[6]=copr(2);
    data_[7]=weightr * 9.80665;
    
    data_[8]=Acc(0);
    data_[9]=Acc(1);
    data_[10]=Acc(2);
    data_[11]=Gyro(0);
    data_[12]=Gyro(1);
    data_[13]=Gyro(2);
    
    data_[14]=joint_states[0];
    data_[15]=joint_states[1];
    data_[16]=joint_states[2];
    data_[17]=joint_states[3];
    data_[18]=joint_states[4];
    data_[19]=joint_states[5];
    data_[20]=joint_states[6];
    data_[21]=joint_states[7];
    data_[22]=joint_states[8];
    data_[23]=joint_states[9];
    data_[24]=joint_states[10];
    data_[25]=joint_states[11];
    data_[26]=joint_states[12];
    data_[27]=joint_states[13];
    data_[28]=joint_states[14];
    data_[29]=joint_states[15];
    data_[30]=joint_states[16];
    data_[31]=joint_states[17];
    data_[32]=joint_states[18];
    data_[33]=joint_states[19];
    data_[34]=joint_states[20];
    data_[35]=joint_states[21];
    data_[36]=joint_states[22];
    data_[37]=joint_states[23];
    data_[38]=joint_states[24];
    data_[39]=joint_states[25];
    fMemoryProxy->raiseEvent("fsr", data_);
    
}
*/

LowLevelPlanner::~LowLevelPlanner()
{
    sockServer.socClose();
    stop();
    
    delete motion;
    delete posture;
    delete tts;
    // Remove the postProcess call back connection
    try{
        fDCMPostProcessConnection.disconnect();
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "reset()", "Error when disconnecting from DCM postProccess: " + e.toString());
    }
    delete fMemoryProxy;
    if (dcm != NULL){
        delete dcm;
        dcm = NULL;
    }
    LOG_INF("Proxies deleted...");
    
    for (int i =0; i< jointPtr.size();i++)
    {
        delete jointPtr[i];
    }
    jointPtr.clear();
    LOG_INF("Joint Pointers deleted...");
    for (int i =0; i< sensorPtr.size();i++)
    {
        delete sensorPtr[i];
    }
    sensorPtr.clear();
    LOG_INF("Sensor Pointers deleted...");
    LOG_INF("Exiting...");
}

void LowLevelPlanner::init()
{
    /** Text To Speech Proxy **/
    tts = new AL::ALTextToSpeechProxy(getParentBroker());
    motion = new AL::ALMotionProxy(getParentBroker());
    posture = new AL::ALRobotPostureProxy(getParentBroker());
    /*
    odom.arraySetSize(8);
    data_.arraySetSize(40);
    debug_data.arraySetSize(9);
    */
    Acc.setZero();
    AccPtr.resize(3,0);
    Gyro.setZero();
    GyroPtr.resize(3,0);
    AnglePtr.resize(3,0);
    Angle.setZero();
    copl.setZero();
    coplPtr.resize(3,0);
    copr.setZero();
    coprPtr.resize(3,0);
    fsrl.setZero();
    joint_statesPtr.resize(26,0);
    fsrr.setZero();
    fsrposl.setZero();
    fsrposr.setZero();

    connectToDevices();
    /**
     Initializing DCM
     **/
    connectToDCM();
    connectToDCMloop();
    start();
    while(1)
    {
//         boost::posix_time::ptime mst1 = boost::posix_time::microsec_clock::local_time();
        if(!sockServer.hasClient())
        {
            LOG_INF("No connecton ");
            sockServer.socAccept();
        }
        else
        {
            if(sendDcmData()==0)
                readCommand();
        }
        
//         boost::posix_time::ptime mst2 = boost::posix_time::microsec_clock::local_time();
//         boost::posix_time::time_duration msdiff = mst2 - mst1;
//         if(msdiff.total_microseconds()<LOOP_TIME)
//             usleep(LOOP_TIME - msdiff.total_microseconds());
        pthread_cond_wait(&cond,&mutex);
        
        if(dcm_state==DCM_READY_TO_STOP)
            stop();
    }
}


void LowLevelPlanner::start()
{
    //Load YAML ideally
    //NaoRobot.readWalkParametersFromFile(defaultFilenameForParameters);
    //NaoRobot.printWalkParameters();
    engine = new WalkEngine(NaoRobot);
   /* 
    try {
        fMemoryProxy->subscribeToEvent("onStepRequested", "LowLevelPlanner", "", "requestStepCallback");
//         fMemoryProxy->subscribeToEvent("onStepReceived","LowLevelPlanner","","stepCallback");
    }
    catch (const AL::ALError& e) {
        qiLogError("LowLevelPlanner") << e.what() << std::endl;
    }
    
    sleep(0.5);    */
    /* 
    /** Text To Speech Proxy **/
    //tts->setLanguage("Greek");
    //tts->setLanguage("English");
    //tts->setParameter("speed", 85);
    
    /**
     Set Body Stiffness
     **/
    motion->setStiffnesses("Body",standingStiffness/2.0);
    motion->setFallManagerEnabled(false);
    motion->wakeUp();
    posture->goToPosture("StandInit",standingStiffness/2.0);
    sleep(0.5);
    
    std::cout<<"Now Initializing DCM Connection "<<std::endl;
    //connectToDCMloop();
    setStiffness(standingStiffness);
    motion->setStiffnesses("Body",standingStiffness);
    dcm_state = DCM_RUNNING;
    
    firstIncomingStep = true;
    
    //Parse the incoming step command
    //Initialization
        engine->addWalkInstruction();
        WalkInstruction i;
        i.targetSupport=KDeviceLists::SUPPORT_LEG_LEFT;
        i.target(0)=NaoRobot.getWalkParameter(HX);
        i.target(1)=-NaoRobot.getWalkParameter(H0);
        i.target(2)=0;
        i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
        i.steps=10*NaoRobot.getWalkParameter(SS_instructions);
        // Adding the Walking Instruction to the Walking Buffer for Execution
        engine->addWalkInstruction(i);
        
        //1 step
        i.targetZMP=KDeviceLists::SUPPORT_LEG_LEFT;
        i.steps=NaoRobot.getWalkParameter(SS_instructions);

        engine->addWalkInstruction(i);
        //2 step
        i.targetSupport=KDeviceLists::SUPPORT_LEG_RIGHT;
        i.targetZMP=KDeviceLists::SUPPORT_LEG_RIGHT;
        i.target(0)=NaoRobot.getWalkParameter(HX);
        i.target(1)=NaoRobot.getWalkParameter(H0);

        engine->addWalkInstruction(i);
        //DS
        i.targetSupport=KDeviceLists::SUPPORT_LEG_RIGHT;
        i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
        engine->addWalkInstruction(i);
        last_step_id=i.step_id;

}

//Sit down motion
void LowLevelPlanner::stop()
{
    dcm_state = DCM_STOP;
    setStiffness(0.6f);
    motion->setStiffnesses("Body",0.6f);
    
    posture->goToPosture("Crouch", 0.15f);
    //-- -- -- -- -- -- -- -- --
    setStiffness(0.0f);
    motion->setStiffnesses("Body",0.0);
    reset();
    if(last_cmd==CROUCH)
    {
        last_cmd_status=STATUS_DONE;
    }
}

void LowLevelPlanner::reset()
{
    LOG_INF("Resetting...");
    //Deallocation!!
    if (engine != NULL){
        delete engine;
        engine = NULL;
    }
    LOG_INF("Walk-Engine deleted...");
}

void LowLevelPlanner::connectToDCMloop()
{
    try
    {
        fDCMPostProcessConnection  = getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&LowLevelPlanner::DCMCallback, this));
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM postProccess: " + e.toString());
    }
    dcm_state = DCM_STOP;
}

int LowLevelPlanner::sendDcmData()
{
    /*
     * Assuming that pre send section is not interapted 
     * or it is interapted by only one dcm loot, then the data is thread safe
     */

    dcm_data_stamp_t *send_data=curr_dcm_data;    

    //PRE_SEND
    if(last_dcm_data_stamp==send_data->stamp) //has alredy been sent
        return 1;
    if(last_dcm_data_stamp>send_data->stamp) //hapens only in case of overflow
        last_dcm_data_stamp=send_data->stamp;
        
    //copy data for thread safety
    dcm_data_stamp_t data=*send_data;    
    //END of PRE_SEND
    
    last_dcm_data_stamp=data.stamp;

    data.dcm_data.last_cmd=last_cmd;
    data.dcm_data.last_cmd_id=last_cmd_id;
    data.dcm_data.last_cmd_param=last_cmd_param;
    if(last_cmd_status!=STATUS_ABORTED)
    {
        if(last_cmd==STEPS)
        {
            if(data.dcm_data.last_footstep>=last_step_id)
            {
             
                LOG_DBG("Steps finished.")
                last_cmd_status=STATUS_DONE;
            }
            else if(dcm_state!=DCM_RUNNING)
            {
                LOG_DBG("Steps aborted.")
                last_cmd_status=STATUS_ABORTED;
            }
            else
            {
                LOG_DBG("Steps pending.")
                last_cmd_status=STATUS_PENDING;
            }
        }
        else if(last_cmd==WAKE_UP)
        {
            if(data.dcm_data.last_footstep>=last_step_id)
                last_cmd_status=STATUS_DONE;
            else
                last_cmd_status=STATUS_PENDING;
        }
    }
    data.dcm_data.last_cmd_status=last_cmd_status;
    
    return sockServer.sendToSoc( (char*) &(data.dcm_data), sizeof(dcm_data_t));
}

int LowLevelPlanner::readCommand()
{
    command_t cmd;
    int ret=sockServer.receiveCommand(cmd);
    if(ret<0)
        return ret;
    //LOG_DBG("New command %d, size:%d",cmd.command,cmd.data);

    if(cmd.command==NONE)
    {
        if(cmd.data!=0)
            free(cmd.data);
        return 0;
    }
    else if(cmd.command==STEPS)
    {
        addCommand(cmd);
        if(NaoRobot.getWalkParameter(velocityControl))
            last_cmd_status=STATUS_ABORTED;        
        if(dcm_state != DCM_RUNNING)
            last_cmd_status=STATUS_ABORTED;
        
        stepsCallback(cmd);
    }
    else if(cmd.command==CROUCH)
    {
        addCommand(cmd);
        if(dcm_state==DCM_RUNNING)
            dcm_state=DCM_NEED_STOP;
        else
            last_cmd_status=STATUS_ABORTED;
    }
    else if(cmd.command==WAKE_UP)
    {
        addCommand(cmd);
        if(dcm_state==DCM_STOP)
        {
            start();
        }
        else
        {
            last_cmd_status=STATUS_ABORTED;
        }
    }
    else if(cmd.command==SPEEK)
    {
        addCommand(cmd);
        if( tts == NULL)
        {
            last_cmd_status=STATUS_ABORTED;
        }
        else
        {
            tts->say(std::string( (char*)cmd.data) );
            last_cmd_status=STATUS_DONE;
        }
    }
    else if(cmd.command==RECONFIG)
    {
        addCommand(cmd);
        if(cmd.data_size != RECONFIG_SIZE*sizeof(float) )
        {
            LOG_ERR("Wrong reconfigure data size.");
            last_cmd_status=STATUS_ABORTED;
        }
        else if(dcm_state!=DCM_RUNNING)
        {
            last_cmd_status=STATUS_ABORTED;
        }
        else
        {
            float *data=(float*)cmd.data;
            setRobotConfig(data);
            last_cmd_status=STATUS_DONE;
        }
    }
    else if(cmd.command==VELOCITY)
    {
        addCommand(cmd);
        if(cmd.data_size != sizeof(velocity_t) )
        {
            LOG_ERR("Wrong velocity data size.");
            last_cmd_status=STATUS_ABORTED;
        }
        else
        {
            velocity_t *data=(velocity_t*)cmd.data;
            velocityCallback(data);
            last_cmd_status=STATUS_DONE;
        }
    }
    else if(cmd.command<BEHAVIOURS_SIZE) //execute behaviour
    {
        addCommand(cmd);        
        
        int behId=cmd.command-COMMANDS_SIZE-2; //Remove alse CROUCH and START
        if(Behaviours::behaviourL.size()<behId-1) //should never happen
        {
            LOG_ERR("Unkwon error %d",last_cmd_status);
            last_cmd_status=STATUS_ABORTED;
        }
        else
        {
            Behaviours::Behaviour beh=Behaviours::behaviourL[behId];
            stop();
            if(runBehavior(beh.behaviour.c_str(),beh.description.c_str())>0)
                last_cmd_status=STATUS_DONE;
            else
                last_cmd_status=STATUS_ABORTED;
        }
    }
    else
    {
        LOG_ERR("Unkwon command %d",cmd.command);        
        last_cmd_status=STATUS_ABORTED;
    }
    if(cmd.data!=0)
        free(cmd.data);
    return 0;
}

/* --------------------------------------------------------------------------------------------------*/




int LowLevelPlanner::DCMCallback()
{   
    // Read Values of joints
    for (int j = 0, i = 0; i < KDeviceLists::NUMOFJOINTS; i++, j++)
    {
        alljoints[j] =*jointPtr[i];
        
    }
    //alljoints[KDeviceLists::R_LEG+KDeviceLists::HIP_YAW_PITCH]=alljoints[KDeviceLists::L_LEG+KDeviceLists::HIP_YAW_PITCH];
    
    // Read IMU
    Acc(0) = -(*AccPtr[0]);
    Acc(1) = *AccPtr[1];
    Acc(2) = -(*AccPtr[2]);
    
    Gyro(0) = *GyroPtr[0];
    Gyro(1) = *GyroPtr[1];
    Gyro(2) = 0.0; //NOT AVAILABLE ATM :(
    
    Angle(0) = *AnglePtr[0];
    Angle(1) = *AnglePtr[1];
    Angle(2) = 0.0; //NOT AVAILABLE ATM :(

    // Read FSR
    for(int i=0; i<4; i++)
    {
        fsrl(i)=*sensorPtr[KDeviceLists::L_FSR+i];
        fsrr(i)=*sensorPtr[KDeviceLists::R_FSR+i];
    }
    
    
    weightl = *weightlPtr;
    weightr = *weightrPtr;
    copl(0) = *coplPtr[0];
    copl(1) = *coplPtr[1];
    copl(2) = 0.0;
    copr(0) = *coprPtr[0];
    copr(1) = *coprPtr[1];
    copr(2) = 0.0;

    for(int i=0;i<26;i++)
        joint_states[i]= *joint_statesPtr[i];
    
    
    
    generateDcmDataEvent();
//     generateOdometryEvent();
//     generateSensorDataEvent();
    //generateDebugDataEvent();

    if (dcm_state == DCM_STOP || dcm_state == DCM_READY_TO_STOP) //Nothing to execute
    {
        return 0;
    }

    if (dcm_state == DCM_NEED_STOP)
    {
        dcm_state = DCM_READY_TO_STOP;
        return 0;
    }
    /** MAIN FUNCTION FOR WALKING **/
    
    if(!engine->isFSRInitialized)
        engine->initFSR(fsrposl,fsrposr);
    
    engine->setFSR(fsrl,fsrr, copl, copr, weightl, weightr);
    engine->setIMU(Acc,Gyro,Angle);
    engine->setJoints(alljoints);  //Feed to kinematics
    
    std::vector<float>joints_action=engine->runStep();
    
    if (joints_action.size() != 12)
    {
        std::cerr<<"Not a Feasible Motion -- Motion Engine Died!"<<std::endl;
        
        int *test;
        test=0;
        *test=5;
        return 0;
    }

    if (dcm_state == DCM_RUNNING)
    {
        //Leg Joint Commands
        int p;
        for (p = 0; p < KDeviceLists::LEG_SIZE * 2; p++)
            commands[5][(p)][0] = (float) joints_action[p];

        //Left Shoulder use right hip value
        commands[5][p+KDeviceLists::SHOULDER_PITCH][0] =engine->armangles(0);
        commands[5][p+KDeviceLists::SHOULDER_ROLL][0] =engine->armangles(1);
        commands[5][p+KDeviceLists::ELBOW_YAW][0] =engine->armangles(2);
        commands[5][p+KDeviceLists::ELBOW_ROLL][0] =engine->armangles(3);
        commands[5][p+KDeviceLists::WRIST_YAW][0]  = 0.0;
        p+=KDeviceLists::ARM_SIZE;
        
        //Right Shoulder use left hip value
        commands[5][p+KDeviceLists::SHOULDER_PITCH][0] = engine->armangles(4);
        commands[5][p+KDeviceLists::SHOULDER_ROLL][0] =engine->armangles(5);
        commands[5][p+KDeviceLists::ELBOW_YAW][0] = engine->armangles(6);
        commands[5][p+KDeviceLists::ELBOW_ROLL][0] =engine->armangles(7);
        commands[5][p+KDeviceLists::WRIST_YAW][0]  = 0.0;
        
        // ======================================================================================================================
        //Send command
        try
        {
            /// Get time in 0 ms -- EXECUTE NOW
            int DCMtime = dcm->getTime(0);
            commands[4][0] = DCMtime;
            dcm->setAlias(commands);
        }
        catch (const AL::ALError &e)
        {
            printf("Error when sending command to DCM !\n\tTrace: %s",e.what());
        }
        
        return 0;
    }
    /*
     else if (dcm_state == DCM_RUNNING_ONLY_FEET)
     {
     //Leg Joint Commands
     int p;
     for (p = 0; p < KDeviceLists::LEG_SIZE * 2; p++)
     commands_onlyFeet[5][(p)][0] = (float) joints_action[p];
     // ======================================================================================================================
     //Send command
     try
     {
     /// Get time in 0 ms -- EXECUTE NOW
     int DCMtime = dcm->getTime(0);
     commands_onlyFeet[4][0] = DCMtime;
     dcm->setAlias(commands_onlyFeet);
     }
     catch (const AL::ALError &e)
     {
     printf("Error when sending command to DCM !\n\tTrace: %s",e.what());
     }
     
     return 0;
     
     
     
     }
     */
    else
        return -1;
}

void LowLevelPlanner::connectToDevices()
{
    
    /*
     try
     {
     dcm_onlyFeet = new AL::DCMProxy(getParentBroker());
     }
     catch (AL::ALError& e)
     {
     printf("Error in getting DCM Proxy!\n\tTrace: %s",e.what());
     }
     */
    
    try
    {
        fMemoryProxy = new  AL::ALMemoryProxy(getParentBroker());
    }
    catch (AL::ALError& e)
    {
        printf("Error in getting ALfMemoryProxy Proxy!\n\tTrace: %s",e.what());
    }

    //Initialise ptr
    std::vector<std::string> jointKeys = KDeviceLists::getJointKeys();
    std::vector<std::string> sensorKeys = KDeviceLists::getSensorKeys();

    /** Computation of the CoP in the Local Coordinate Frame of the Foot **/
    try {
        fsrposl(0,0)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/XPosition");
        fsrposl(0,1)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/XPosition");
        fsrposl(0,2)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/XPosition");
        fsrposl(0,3)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/XPosition");
        
        fsrposr(0,0)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/XPosition");
        fsrposr(0,1)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/XPosition");
        fsrposr(0,2)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/XPosition");
        fsrposr(0,3)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/XPosition");
        
        fsrposl(1,0)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/YPosition");
        fsrposl(1,1)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/YPosition");
        fsrposl(1,2)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/YPosition");
        fsrposl(1,3)=fMemoryProxy->getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/YPosition");
        
        fsrposr(1,0)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/YPosition");
        fsrposr(1,1)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/YPosition");
        fsrposr(1,2)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/YPosition");
        fsrposr(1,3)=fMemoryProxy->getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/YPosition");        
        
        // Read IMU
        AccPtr[0] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value");
        AccPtr[1] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value");
        AccPtr[2] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value");
        
        GyroPtr[0] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value");
        GyroPtr[1] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value");
        GyroPtr[2] = NULL; //NOT AVAILABLE ATM :(
        
        AnglePtr[0] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
        AnglePtr[1] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
        AnglePtr[2] = NULL; //NOT AVAILABLE ATM :(
        
        weightlPtr = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LFoot/FSR/TotalWeight/Sensor/Value");
        weightrPtr = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RFoot/FSR/TotalWeight/Sensor/Value");
        coplPtr[0] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/X/Sensor/Value");
        coplPtr[1] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LFoot/FSR/CenterOfPressure/Y/Sensor/Value");
        coplPtr[2] = NULL;
        coprPtr[0] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/X/Sensor/Value");
        coprPtr[1] = (float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RFoot/FSR/CenterOfPressure/Y/Sensor/Value");
        coprPtr[2] = NULL;
        
        joint_statesPtr[0]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
        joint_statesPtr[1]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
        joint_statesPtr[2]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
        joint_statesPtr[3]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
        joint_statesPtr[4]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
        joint_statesPtr[5]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
        joint_statesPtr[6]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
        joint_statesPtr[7]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LHand/Position/Sensor/Value");
        joint_statesPtr[8]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
        joint_statesPtr[9]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
        joint_statesPtr[10]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
        joint_statesPtr[11]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
        joint_statesPtr[12]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
        joint_statesPtr[13]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
        joint_statesPtr[14]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
        joint_statesPtr[15]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
        joint_statesPtr[16]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
        joint_statesPtr[17]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
        joint_statesPtr[18]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
        joint_statesPtr[19]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
        joint_statesPtr[20]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
        joint_statesPtr[21]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
        joint_statesPtr[22]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
        joint_statesPtr[23]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
        joint_statesPtr[24]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");
        joint_statesPtr[25]=(float *) fMemoryProxy->getDataPtr("Device/SubDeviceList/RHand/Position/Sensor/Value");
        
        
    }
    catch (const AL::ALError& e) 
    {
        printf("Could not get Pressure Data from NAO.\n\tTrace: %s",
               e.what());
        return;
    }
    
    sensorPtr.resize(KDeviceLists::NUMOFSENSORS);
    for (int i = 0; i < KDeviceLists::NUMOFSENSORS; i++)
    {
        sensorPtr[i] = (float *) fMemoryProxy->getDataPtr(sensorKeys[i]);
    }
    
    jointPtr.resize(KDeviceLists::NUMOFJOINTS);
    for (int i = 0; i < KDeviceLists::NUMOFJOINTS; i++)
    {
        jointPtr[i] = (float *) fMemoryProxy->getDataPtr(jointKeys[i]);
    }
    
    joint_states.resize(26,0);
    alljoints.resize(KDeviceLists::NUMOFJOINTS, 0);

    sleep(1.0);
}

void LowLevelPlanner::connectToDCM()
{
    try
    {
        dcm = new AL::DCMProxy(getParentBroker());
    }
    catch (AL::ALError& e)
    {
        printf("Error in getting DCM Proxy!\n\tTrace: %s",e.what());
    }
    
    createHardnessActuatorAlias();
    prepareJointsPositionActuatorCommand();
    //prepareJointsPositionActuatorCommand_onlyFeet();
    createJointsPositionActuatorAlias();
    //createJointsPositionActuatorAlias_onlyFeet();
    sleep(1);
    std::cout<<"Robot's Devices initialized"<<std::endl;
}

void LowLevelPlanner::prepareJointsPositionActuatorCommand()
{
    commands.arraySetSize(6);
    commands[0] = std::string("jointActuator");
    commands[1] = std::string("ClearAll"); /// Erase all previous commands
    commands[2] = std::string("time-separate");
    commands[3] = 0;
    commands[4].arraySetSize(1);
    commands[5].arraySetSize(KDeviceLists::LEG_SIZE * 2 +KDeviceLists::ARM_SIZE * 2); // For joints //2legs + 2 hip_pitch
    
    for (int i = 0; i < KDeviceLists::LEG_SIZE * 2 + KDeviceLists::ARM_SIZE * 2; i++)
        commands[5][i].arraySetSize(1);
    
}

void LowLevelPlanner::createJointsPositionActuatorAlias()
{
    AL::ALValue jointAliasses;
    
    std::vector<std::string> jointActuatorKeys = KDeviceLists::getPositionActuatorKeys();
    
    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointActuator"); // Alias for all joint actuators
    
    jointAliasses[1].arraySetSize(KDeviceLists::LEG_SIZE * 2 + KDeviceLists::ARM_SIZE * 2); //
    
    //int idx = 0;
    // Joints actuator list
    std::string actuatorname;
    int l = 0;
    for (int j = KDeviceLists::HIP_YAW_PITCH; j < KDeviceLists::LEG_SIZE; j++, l++)
    {
        actuatorname = jointActuatorKeys[KDeviceLists::L_LEG + j];
        jointAliasses[1][l] = actuatorname;
        //std::cout << " Joint Name " << actuatorname << " " << std::endl;
    }
    
    for (int j = KDeviceLists::HIP_YAW_PITCH; j < KDeviceLists::LEG_SIZE; j++, l++)
    {
        actuatorname = jointActuatorKeys[KDeviceLists::R_LEG + j];
        jointAliasses[1][l] = actuatorname;
        //std::cout << " Joint Name " << actuatorname << " " << std::endl;
    }
    
    for (int j = 0; j <  KDeviceLists::ARM_SIZE; j++, l++)
    {
        actuatorname = jointActuatorKeys[KDeviceLists::L_ARM + j];
        jointAliasses[1][l] = actuatorname;
        //std::cout << " Joint Name " << actuatorname << " " << std::endl;
    }
    
    for (int j = 0; j <  KDeviceLists::ARM_SIZE; j++, l++)
    {
        actuatorname = jointActuatorKeys[KDeviceLists::R_ARM + j];
        jointAliasses[1][l] = actuatorname;
        //std::cout << " Joint Name " << actuatorname << " " << std::endl;
    }
    
    
    ///*Create Joint Alias*
    try
    {
        dcm->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        //throw ALERROR("mainModule", "createPositionActuatorAlias()", "Error when creating Alias : " + e.toString());
        printf("Error when creating DCM Alias!\n\tTrace: %s",e.what());
    }
    
    std::cout<<"PositionActuatorAlias created "<<std::endl;
}

void LowLevelPlanner::setStiffness(const float& stiffnessValue)
{
    setStiffnessDCM(stiffnessValue);
}

void LowLevelPlanner::createHardnessActuatorAlias()
{
    AL::ALValue jointAliasses;
    // Alias for all joint stiffness
    jointAliasses.clear();
    
    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointStiffness"); // Alias for all 25 actuators
    jointAliasses[1].arraySetSize(KDeviceLists::NUMOFJOINTS);
    //std::cout <<"size " <<  KDeviceLists::NUMOFJOINTS << std::endl;
    // stiffness list
    std::vector<std::string> HardnessActuatorStrings = KDeviceLists::getHardnessActuatorKeys();
    // Joints actuator list
    for (int i = 0; i < KDeviceLists::NUMOFJOINTS; i++)
    {
        jointAliasses[1][i] = HardnessActuatorStrings[i];
    }
    // Create alias
    try
    {
        dcm->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        printf("Error when creating Hardness Alias!\n\tTrace: %s",e.what());
    }
}

void LowLevelPlanner::setStiffnessDCM(const float &stiffnessValue)
{
    AL::ALValue stiffnessCommands;
    int DCMtime;
    // increase stiffness with the "jointStiffness" Alias created at initialization
    try
    {
        // Get time : return the time in 1 second
        DCMtime = dcm->getTime(1000);
    }
    catch (const AL::ALError &e)
    {
        printf("Error on DCM getTime!\n\tTrace: %s",e.what());
    }
    
    // Prepare one dcm command:
    // it will linearly "Merge" all joint stiffness
    // from last value to "stiffnessValue" in 1 second
    stiffnessCommands.arraySetSize(3);
    stiffnessCommands[0] = std::string("jointStiffness");
    stiffnessCommands[1] = std::string("ClearAll");
    stiffnessCommands[2].arraySetSize(1);
    stiffnessCommands[2][0].arraySetSize(2);
    stiffnessCommands[2][0][0] = stiffnessValue;
    stiffnessCommands[2][0][1] = DCMtime;
    try
    {
        dcm->set(stiffnessCommands);
    }
    catch (const AL::ALError &e)
    {
        printf("Error when sending stiffness to DCM!\n\tTrace: %s",e.what());
    }
}


/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/
/* --------------------------------------------------------------------------------------------------*/

void LowLevelPlanner::prepareJointsPositionActuatorCommand_onlyFeet()
{
    commands_onlyFeet.arraySetSize(6);
    commands_onlyFeet[0] = std::string("jointActuator");
    commands_onlyFeet[1] = std::string("ClearAll"); /// Erase all previous commands
    commands_onlyFeet[2] = std::string("time-separate");
    commands_onlyFeet[3] = 0;
    commands_onlyFeet[4].arraySetSize(1);
    commands_onlyFeet[5].arraySetSize(KDeviceLists::LEG_SIZE * 2 );
    
    for (int i = 0; i < KDeviceLists::LEG_SIZE * 2; i++)
        commands_onlyFeet[5][i].arraySetSize(1);
    
}

void LowLevelPlanner::createJointsPositionActuatorAlias_onlyFeet()
{
    AL::ALValue jointAliasses;
    
    std::vector<std::string> jointActuatorKeys = KDeviceLists::getPositionActuatorKeys();
    
    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointActuator"); // Alias for all joint actuators
    
    jointAliasses[1].arraySetSize(KDeviceLists::LEG_SIZE * 2); //
    
    //int idx = 0;
    // Joints actuator list
    std::string actuatorname;
    int l = 0;
    for (int j = KDeviceLists::HIP_YAW_PITCH; j < KDeviceLists::LEG_SIZE; j++, l++)
    {
        actuatorname = jointActuatorKeys[KDeviceLists::L_LEG + j];
        jointAliasses[1][l] = actuatorname;
        std::cout << " Joint Name " << actuatorname << " " << std::endl;
    }
    
    for (int j = KDeviceLists::HIP_YAW_PITCH; j < KDeviceLists::LEG_SIZE; j++, l++)
    {
        actuatorname = jointActuatorKeys[KDeviceLists::R_LEG + j];
        jointAliasses[1][l] = actuatorname;
        std::cout << " Joint Name " << actuatorname << " " << std::endl;
    }
    
    ///*Create Joint Alias*
    try
    {
        dcm_onlyFeet->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        //throw ALERROR("mainModule", "createPositionActuatorAlias()", "Error when creating Alias : " + e.toString());
        printf("Error when creating DCM Alias!\n\tTrace: %s",e.what());
    }
    
    std::cout<<"PositionActuatorAlias created "<<std::endl;
}
//0 1 2 3

int LowLevelPlanner::getDCMState()
{
    return dcm_state;
}

void LowLevelPlanner::setDCMState(int st_)
{
    if(st_>=0 && st_<=3)
        dcm_state = st_;
    else
        std::cout<<"Error Setting DCM State to "<<st_<<std::endl;
}

AL::ALValue LowLevelPlanner::getFootSteps()
{
    if(engine != NULL)
    {
        AL::ALValue data;
        data.arraySetSize(5);
        data[0]=engine->executedStep.target(0);
        data[1]=engine->executedStep.target(1);
        data[2]=engine->executedStep.target(2);
        if(engine->executedStep.targetSupport==KDeviceLists::SUPPORT_LEG_LEFT)
            data[3]="rleg";
        else
            data[3]="lleg";
        
        data[5]=engine->executedStep.step_id;
        return data;
    }
}

#if 0
int LowLevelPlanner::setFootSteps(AL::ALValue value)
{

    int id = -1;
    if(engine != NULL)
    {
        /** Initial Walking Instruction **/
        WalkInstruction i;
        
        /** Randomly chosen Left Leg as Initial Support Leg **/
        std::string leg_id = value[3];
        if(leg_id =="lleg")
            i.targetSupport=KDeviceLists::SUPPORT_LEG_RIGHT;
        else
            i.targetSupport=KDeviceLists::SUPPORT_LEG_LEFT;
        
        
        i.target(0) = value[0];
        i.target(1) = value[1];
        i.target(2) = value[2];
        /** ZMP in the Middle of Convex Hull **/
        std::string wtd = value[4];
        if(wtd=="stand")
            i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
        else
            i.targetZMP=i.targetSupport;
        i.steps=NaoRobot.getWalkParameter(SS_instructions);
        i.step_id = engine->assignStepID();
        id = i.step_id;
        engine->stepAnkleQ.push_back(i);
    }
    return id;
}
#endif

// void LowLevelPlanner::requestStepCallback(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg) {
//     qiLogInfo("LowLevelPlanner") << "requestStepCallback:" << key << std::endl;
//     qiLogInfo("LowLevelPlanner") << "Value   :" << value << std::endl;
// }


void LowLevelPlanner::stepsCallback(const command_t &cmd)
{    
    step_t *steps=(step_t*)cmd.data;    
    int step_size=cmd.data_size/sizeof(step_t);    
    LOG_DBG("Received %d steps.",step_size);
    for(int j=0;j<step_size;j++)
    {        
        /** Initial Walking Instruction **/
        WalkInstruction i;

        /** Randomly chosen Left Leg as Initial Support Leg **/
        if(steps[j].leg ==LEFT)
            i.targetSupport=KDeviceLists::SUPPORT_LEG_RIGHT;
        else
            i.targetSupport=KDeviceLists::SUPPORT_LEG_LEFT;

        i.target(0) = steps[j].x;
        i.target(1) = steps[j].y;
        i.target(2) = steps[j].theta;
        /** ZMP in the Middle of Convex Hull **/
        if( steps[j].cmd==STAND)
            i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
        else
            i.targetZMP=i.targetSupport;
        i.steps=NaoRobot.getWalkParameter(SS_instructions);
        
        std::cout<<"step "<<i.target(0)<<" "<<i.target(1)<<" "<<i.target(2)<<" "<<i.targetSupport<<" "<<i.targetZMP<<endl;
        engine->addWalkInstruction(i);

        if(j==0)
            first_step_id=i.step_id;
        else if(j==step_size-1)
            last_step_id=i.step_id;
        
    }
}

/*
void LowLevelPlanner::stepCallback(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg) {
    //   std::cout << "stepCallback:" << key << std::endl;
    //   std::cout << "Step X   :" << value[0] << std::endl;
    //   std::cout << "Step Y   :" << value[1] << std::endl;
    //   std::cout << "Step Theta   :" << value[2] << std::endl;
    //   std::cout << "Leg id   :" << value[3] << std::endl;
    //   std::cout << "What To Do   :" << value[4] << std::endl;
    
    
    
    if(engine != NULL)
    {
       
        WalkInstruction i;
        
       
        std::string leg_id = value[3];
        if(leg_id =="lleg")
            i.targetSupport=KDeviceLists::SUPPORT_LEG_RIGHT;
        else
            i.targetSupport=KDeviceLists::SUPPORT_LEG_LEFT;
        
        
        i.target(0) = value[0];
        i.target(1) = value[1];
        i.target(2) = value[2];
        
        std::string wtd = value[4];
        if(wtd=="stand")
            i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
        else
            i.targetZMP=i.targetSupport;
        i.steps=NaoRobot.getWalkParameter(SS_instructions);
        i.step_id = engine->assignStepID();
        engine->stepAnkleQ.push_back(i);
    }
    
}
*/

//Function for Running A CRG File
int LowLevelPlanner::runBehavior(const char* behaviorPath, const char* behaviorDescription)
{
    
    /*
     std::vector<std::string> installedBehs = behavior.getInstalledBehaviors();
     for (int i=0; i<installedBehs.size(); i++)
     {
     fprintf(stderr,"Behavior %u = %s \n" , i ,installedBehs[i].c_str());
     }
     */
    if ( behavior.isBehaviorInstalled(behaviorPath) )
    {
        if (strlen(behaviorDescription)>0)
        {
            tts->say(behaviorDescription);
        }
        else
        {
            LOG_DBG("A silent behavior");
        }
        behavior.runBehavior(behaviorPath);//Blocking Call to start behavior
        //tts->say("Καλά τα πήγα!");
        //sleep(1);
        behavior.stopAllBehaviors();
        sleep(1);
        return 1;
    }
    else
    {
        tts->say("Δεν ξέρω πως να το κάνω αυτό");
        return -1;
        
    }
}

void LowLevelPlanner::setRobotConfig(float *value)
{
    if(engine != NULL){
        
        engine->NaoLIPM.Observer_COPX = value[0];
        engine->NaoLIPM.Observer_COPY = value[1];
        engine->NaoLIPM.Observer_CoMX = value[2];
        engine->NaoLIPM.Observer_CoMY = value[3];
        
        // engine->NaoPosture.Kp_PitchT = value[4];
        // engine->NaoPosture.Kd_PitchT = value[5];
        engine->NaoVRPToCoM.K_pitch = value[4];
        engine->NaoVRPToCoM.K_roll = value[6];

        // engine->NaoPosture.Kp_RollT = value[6];
        // engine->NaoPosture.Kd_RollT = value[7];
        engine->NaoPosture.amX = value[8];
        engine->NaoPosture.amY = value[9];
        //engine->NaoFeetEngine.StepZ_ = value[10];
        engine->NaoVRPToCoM.gain_x = value[11];
        engine->NaoVRPToCoM.gain_y = value[12];
        std::cout<<"Config change"<<std::endl;
    }
}

void LowLevelPlanner::velocityCallback(velocity_t *data)
{
     std::cout<<"Velocity:"<<data->vx<<","<<data->vy<<","<<data->vw<<std::endl;
     engine->AddVelocityInstruction(KVecFloat3(data->vx,data->vy,data->vw));

}

bool LowLevelPlanner::sayText(AL::ALValue value)
{
    if( tts != NULL){
        std::string text = value;
        tts->say(text);
        return true;
    }
    else
        return false;
    
}
