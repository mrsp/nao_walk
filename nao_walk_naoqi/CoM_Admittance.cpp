#include "CoM_Admittance.h"
CoM_Admittance::CoM_Admittance(RobotParameters &robot):NaoRobot(robot)
{
    K = NaoRobot.getWalkParameter(CoM_Stiffness);
    D = NaoRobot.getWalkParameter(CoM_Damping);
    M = NaoRobot.getWalkParameter(mass);
    dt = NaoRobot.getWalkParameter(Ts);
    e.zero();
    A(0,0) = 1.00;
    A(0,1) = dt;
    A(1,0) = -K/M * dt;
    A(1,1) = 1-D/M * dt;
    B(0) = 0;
    B(1) = dt / M ;
    
    
    
    std::cout<<"CoM Admitance Controller Initialized Successfully"<<std::endl;
    resetted = true;
    
}

void CoM_Admittance::reset()
{
    
    e.zero();
    resetted = true;
    std::cout<<"CoM Admitance Controller Reseted"<<std::endl;
    
    
    
}


float CoM_Admittance::Control(float force)
{
    if(resetted)
        resetted = false;
    //Spring and damper on CoM
    e = A * e;
    temp = B;
    temp.scalar_mult(force-27.5004);
    e += temp;
    
    
    return e(0);
    
}
