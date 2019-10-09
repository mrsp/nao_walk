#include "JointSSKF.h"



void JointSSKF::initialize(RobotParameters &robot)
{

    NaoRobot=robot;


    F.zero();
    F(0,0)=1.000;
    F(0,1)=NaoRobot.getWalkParameter(Ts);
    F(1,1)=1.000;
    x.zero();
    K.zero();
    K(0)=0.4738;
    K(1)=0.9176;


    JointPosition=0.000;
    JointVelocity=0.000;

    firstrun=true;

    std::cout<<"Joint Steady-State Kalman Filter Initialized Successfully"<<std::endl;


}


void JointSSKF::reset()
{

    F.zero();
    F(0,0)=1.000;
    F(0,1)=NaoRobot.getWalkParameter(Ts);
    F(1,1)=1.000;

    x.zero();


    K.zero();
    K(0)=0.4738;
    K(1)=0.9176;


    JointPosition=0.000;
    JointVelocity=0.000;

    firstrun=true;

    std::cout<<"Joint Steady-State Kalman Filter Reseted"<<std::endl;


}




/** JointSSKF filter to  deal with Delay, and  Noise **/
void JointSSKF::Filter(float JointPosMeasurement)
{

    if(firstrun)
    {
        x(0)=JointPosMeasurement;
        firstrun=false;
    }

    /** Predict **/

    x=F*x;

    /** Update **/

    x+=K*(JointPosMeasurement-x(0));


    JointPosition=x(0);
    JointVelocity=x(1);
    /** ------------------------------------------------------------- **/
}
