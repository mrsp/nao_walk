/*! \file LowLevelPlanner.h
 *	\brief A Monas Activity that first plans the trajectories needed by the Walk Engine
 and executes the desired walking gait!
 *
 */

#ifndef LOWLEVELPLANNER_H
#define LOWLEVELPLANNER_H
#include <iostream>
#include <string>
#include <vector>
#include "RobotParameters.h"
#include "WalkEngine.h"
#include "MotionDefines.h"
// NAOqi Headers
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/albehaviormanagerproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alvalue/alvalue.h>
#include <alproxies/altouchproxy.h>
//Vision/Camera
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/alrobotpostureproxy.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/dcmproxy.h>
#include <qi/os.hpp>
#include <alerror/alerror.h>

// Boost Headers
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <unistd.h>
#include <signal.h>
#include <fstream>

#include"SocketServer.h"
#include"DcmData.h"
#include <pthread.h>

enum
{
    DCM_RUNNING, DCM_RUNNING_ONLY_FEET,DCM_NEED_STOP, DCM_READY_TO_STOP, DCM_STOP, 
};




/**
 * \class LowLevelPlanner
 *
 * \file LowLevelPlanner.h
 **/

namespace AL
{
    class ALBroker;
}


template<typename T, size_t N>
T * end(T (&ra)[N])

 {
     return ra + N;
 }


class LowLevelPlanner : public AL::ALModule
{
private:
//     bool stopEngine;
    std::list<WalkInstruction> stepList;
    SocketServer sockServer;
    pthread_cond_t cond;
    pthread_mutex_t mutex;
    // Used for postprocess sync with the DCM
    ProcessSignalConnection fDCMPostProcessConnection;
    /** Initialise the DCM part **/
    void connectToDevices();
    
    /** Connect To the DCM loop 100Hz **/
    void connectToDCMloop();
    
    /** function bound to the DCM **/
    int DCMCallback();
    
    /** Create DCM hardness Actuator Alias **/
    void createHardnessActuatorAlias();
    
    /** Create DCM Position Actuator Alias **/
    void createJointsPositionActuatorAlias();
    
    /** Prepare Command ALValue to send command to actuator **/
    void prepareJointsPositionActuatorCommand();
    
    /** Create DCM Position Actuator Alias **/
    void createJointsPositionActuatorAlias_onlyFeet();
    
    /** Prepare Command ALValue to send command to actuator **/
    void prepareJointsPositionActuatorCommand_onlyFeet();
    
    /** Set Stiffness with the DCM **/
    void setStiffnessDCM(const float &stiffnessValue);
    
    int first_step_id;
    int last_step_id;
    /**
     Used by DCM callbacks
     **/
    std::vector<float *> jointPtr, sensorPtr;
    //boost::shared_ptr<AL::ALMemoryProxy> memory;
    AL::ALMemoryProxy *fMemoryProxy;
    AL::ALTextToSpeechProxy *tts;
    AL::DCMProxy *dcm, *dcm_onlyFeet;
    AL::ALMotionProxy *motion;
    AL::ALRobotPostureProxy *posture;
    std::vector<float> alljoints, joint_states;
    std::vector<float*>joint_statesPtr, AnglePtr, AccPtr, GyroPtr;
    std::vector<float*>coprPtr, coplPtr;

    float *weightlPtr, *weightrPtr;
    
    AL::ALValue commands, commands_onlyFeet;
//     AL::ALValue odom;
//     AL::ALValue data_;
    AL::ALValue debug_data;
    
        dcm_data_stamp_t dcm_data[2];
    dcm_data_stamp_t *curr_dcm_data;
    
    int last_dcm_data_stamp;
    int last_cmd;
    unsigned int last_cmd_id;
    int last_cmd_param;
    int last_cmd_status;
    
    /****/
    Matrix<float, 3, 4> fsrposl, fsrposr;
    Vector4f fsrl,fsrr;
    Vector3f Angle, Acc, Gyro;
    Vector3f copr, copl;
    
    
    float weightr, weightl;
    int  dcm_state;
    bool firstIncomingStep;
    void reset();

    bool readyToStop, readyToStand, readyToReset;
    AL::ALBehaviorManagerProxy behavior;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief
     **/
    LowLevelPlanner(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
    ~LowLevelPlanner();
//     int isFootStepExecuted();
    AL::ALValue getFootSteps();
//     int setFootSteps(AL::ALValue value);
    
    /**
     * Callback function every time the example event is called.
     */
    void stepsCallback(const command_t &cmd);
    
//     void requestStepCallback(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg);
//     void stepCallback(const std::string &key, const AL::ALValue &value, const AL::ALValue &msg);
    
    /**
     * A very simple function to generate the event.
     */
    void generateDcmDataEvent();    
    void generateDebugDataEvent();
//     void generateOdometryEvent();
//     void generateSensorDataEvent();
    void setRobotConfig(float* value);
    void velocityCallback(velocity_t *data);
    void init();
    void start();
    void stop();
//     int  execute();
    void connectToDCM();
    
     int  sendDcmData();
    int readCommand();
    
    void setDCMState(int st_);
    int  getDCMState();
    bool sayText(AL::ALValue value);
    
    int runBehavior(const char * behaviorPath , const char * behaviorDescription);
    
    void inline addCommand(const command_t &cmd)
    {
        last_cmd=cmd.command;
        last_cmd_id=cmd.id;
        last_cmd_param=PARAM_INV;
        last_cmd_status=STATUS_PENDING;
    }
    /**
     Main object instances used by KWalk
     **/
    RobotParameters NaoRobot;
    WalkEngine* 	engine;
    
    
    /** Set one hardness value to all Body joints **/
    void setStiffness(const float &stiffnessValue);
};
#endif

