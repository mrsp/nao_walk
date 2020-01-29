#include "ZMPFilter.h"


ZMPFilter::ZMPFilter(RobotParameters &robot):NaoRobot(robot)
{    
    Ib.setZero();  
    Ib(0,0) = NaoRobot.getWalkParameter(I_xx);
    Ib(1,1) = NaoRobot.getWalkParameter(I_yy);
    Ib(2,2) = NaoRobot.getWalkParameter(I_zz);
    alpha = NaoRobot.getWalkParameter(alpha_ZMPFilter);
    m = NaoRobot.getWalkParameter(mass);
    gravity = NaoRobot.getWalkParameter(g);
    cout<<" ZMP Filter Initialized"<<endl;
}



void ZMPFilter::reset()
{
    alpha = NaoRobot.getWalkParameter(alpha_ZMPFilter);
    m = NaoRobot.getWalkParameter(mass);
    gravity = NaoRobot.getWalkParameter(g);
    Ib.setZero();  
    Ib(0,0) = NaoRobot.getWalkParameter(I_xx);
    Ib(1,1) = NaoRobot.getWalkParameter(I_yy);
    Ib(2,2) = NaoRobot.getWalkParameter(I_zz);
    cout<<" ZMP Filter Reseted"<<endl;   
}




/** ZMPFilter filter to  deal with the Noise **/
Vector3f ZMPFilter::filter(Vector3f  cop, Vector3f CoM, Vector3f Acc, Vector3f Gyro, Vector3f Gyrodot, Matrix3f Rotwb)
{
    Vector3f out, L;
    Acc += (Rotwb*Gyro).cross( (Rotwb*Gyro).cross(CoM)) + (Rotwb*Gyrodot).cross(CoM);
    L = Rotwb*Ib * Gyrodot + (Rotwb*Gyro).cross(Rotwb*Ib*Gyro);

    out(0) = alpha * cop(0) + (1.0-alpha) * (CoM(0) - Acc(0)*(CoM(2)-cop(2))/(Acc(2) + gravity) - L(1)/(m*(Acc(2) + gravity)));
    out(1) = alpha * cop(1) + (1.0-alpha) * (CoM(1) - Acc(1)*(CoM(2)-cop(2))/(Acc(2) + gravity) + L(0)/(m*(Acc(2) + gravity)));
    out(2) = cop(2);

    return out;
    
    
    /** ------------------------------------------------------------- **/
}
