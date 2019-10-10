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

    Ccom(0,0) = 1.000;
	Czmp(0,2) = 1.000;
	Czmp(0,3) = 1.000;
    



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
L(0,0) =0.16942;
L(0,1) =-0.0027512;
L(1,0) =0.36788;
L(1,1) =-0.045975;
L(2,0) =-0.024534;
L(2,1) =0.31559;
L(3,0) =0.024521;
L(3,1) =0.12636;

    L=L*0;
    
   Lcom.setZero();
   Lcom  = L.block<4,1>(0,0);


    cout<<"ZMP Delayed Observer Initialized Successfully"<<endl;
    firstrun = true;

}

void DelayedObserverDCM::setInitialState(Vector4f x_)
{
    x = x_;
    updateVars();
    firstrun = false;
}


void DelayedObserverDCM::update(float u_, float zmp_, float com_)
{
    
    if(xbuffer.size() > (int) ZMPDELAY - 1)
    {
        com_ -= Ccom*x;
        zmp_ -= Czmp*xbuffer.front();
        x = A*x;
        x.noalias() +=  B*u_;
        x.noalias() +=  L * Vector2f(com_,zmp_);
        xbuffer.pop();        
    }
    else
    {
        com_ -= Ccom*x;
        x = A*x;
        x.noalias() +=  B*u_;
        x.noalias() +=  Lcom * com_;
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