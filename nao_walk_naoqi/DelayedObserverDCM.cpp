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
L(0,0) =1.2912;
L(0,1) =0.062118;
L(0,2) =0.012288;
L(1,0) =0.011382;
L(1,1) =1.0712;
L(1,2) =0.089333;
L(2,0) =-0.064134;
L(2,1) =3.6675;
L(2,2) =-0.32128;
L(3,0) =0.22132;
L(3,1) =0.31725;
L(3,2) =2.4241;

L = L*0;
    Lcom.setZero();
    Lcom  = L.block<4,2>(0,0);
    cout<<"DCM Delayed Observer Initialized Successfully"<<endl;
    firstrun = true;
    Observer_CoM = 0.01;
    Observer_DCM = 0.01;
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
    if(fabs(zmp_)>1e-10)
    {
        com_ -= Ccom*x;
        zmp_ -= Czmp*x;
        dcm_ -= Cdcm*x;

        com_ = com_ * Observer_CoM;
        dcm_ = dcm_ * Observer_DCM;
        zmp_ = zmp_ * Observer_COP;
        x = A*x;
        x.noalias() +=  B*u_;
        x.noalias() +=  L * Vector3f(com_,dcm_,zmp_);
    }
    else
    {
        com_ -= Ccom*x;
        dcm_ -= Cdcm*x;
        com_ = com_ * Observer_CoM;
        dcm_ = dcm_ * Observer_DCM;
        x = A*x;
        x.noalias() +=  B*u_;
        x.noalias() +=  Lcom * Vector2f(com_,dcm_);
    }
    
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