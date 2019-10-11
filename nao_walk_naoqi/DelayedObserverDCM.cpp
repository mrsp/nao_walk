#include "DelayedObserverDCM.h"

DelayedObserverDCM::DelayedObserverDCM(RobotParameters &robot):NaoRobot(robot)
{

    
    //State is com, dcm, vrp, and ZMP offset
    x.setZero();
    A.setZero();
    B.setZero();
    I.setIdentity();
 
    
    Ccom.setZero();
    Czmp.setZero();
    Cdcm.setZero();

    Ccom(0,0) = 1.000;
	Czmp(0,2) = 1.000;
	Czmp(0,3) = 1.000;
    Cdcm(0,1) = 1,000;



    A(0,0) = -NaoRobot.getWalkParameter(omega);
    A(0,1) = NaoRobot.getWalkParameter(omega);
    A(1,1) = NaoRobot.getWalkParameter(omega);
    A(1,2) = -NaoRobot.getWalkParameter(omega);
    A *= NaoRobot.getWalkParameter(Ts);
    A.noalias() += I;
 
    B.setZero();
    B(2) = 1.000;
    B *= NaoRobot.getWalkParameter(Ts);


    L.setZero();

L(0,0) =0.0075179;
L(0,1) =0.00020371;
L(0,2) =0.0011332;
L(1,0) =3.0235e-05;
L(1,1) =0.0011421;
L(1,2) =0.00044128;
L(2,0) =0.00026634;
L(2,1) =0.015492;
L(2,2) =-0.039449;
L(3,0) =-0.00078774;
L(3,1) =-0.00070665;
L(3,2) =0.027235;
L = L * 0;
    Lcom.setZero();
    Lcom  = L.block<4,2>(0,0);
    cout<<"DCM Delayed Observer Initialized Successfully"<<endl;
    firstrun = true;
    Observer_CoM = 0.01;
    Observer_COP = 0.01;
}

void DelayedObserverDCM::setInitialState(Vector4f x_)
{
    x = x_;
    updateVars();
    firstrun = false;
}


void DelayedObserverDCM::update(float u_, float zmp_, float dcm_, float com_)
{  
    if(xbuffer.size() > (int) ZMPDELAY - 1)
    {
        com_ -= Ccom*x;
        zmp_ -= Czmp*xbuffer.front();
        dcm_ -= Cdcm*x;

        com_ = com_ * Observer_CoM;
        dcm_ = dcm_ * Observer_CoM;
        zmp_ = zmp_ * Observer_COP;
        x = A*x;
        x.noalias() +=  B*u_;
        x.noalias() +=  L * Vector3f(com_,dcm_,zmp_);
        xbuffer.pop();        
    }
    else
    {
        com_ -= Ccom*x;
        dcm_ -= Cdcm*x;
        x = A*x;
        x.noalias() +=  B*u_;
        x.noalias() +=  Lcom * Vector2f(com_,dcm_);
    }
    xbuffer.push(x);
    updateVars();
}



void DelayedObserverDCM::updateVars()
{
    com = x(0);
    dcm = x(1);
    vrp = x(2);
    dist = x(3);
}

Vector4f DelayedObserverDCM::getState()
{
    return x;
}