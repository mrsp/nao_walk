#include "DelayedKalmanDCM.h"

DelayedKalmanDCM::DelayedKalmanDCM(RobotParameters &robot):NaoRobot(robot),kbuffer(10)
{

    
    //State is com, dcm, vrp, and ZMP offset
    x.setZero();
    A.setZero();
    B.setZero();
    Q.setZero();
    I.setIdentity();
    M.setIdentity();
    P.setZero();
    P(0,0)=0.005;
    P(1,1)=0.01;
    P(2,2)=0.1;
    P(3,3)=1.0;
    
    K.setZero();
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


    
    Q(0,0) = NaoRobot.getWalkParameter(CoM_state_uncertainty) * NaoRobot.getWalkParameter(CoM_state_uncertainty);
    Q(1,1) = NaoRobot.getWalkParameter(DCM_state_uncertainty) * NaoRobot.getWalkParameter(DCM_state_uncertainty);
    Q(2,2) = NaoRobot.getWalkParameter(VRP_state_uncertainty) * NaoRobot.getWalkParameter(VRP_state_uncertainty);
    Q(3,3) = NaoRobot.getWalkParameter(External_wrench_uncertainty) * NaoRobot.getWalkParameter(External_wrench_uncertainty);
    
    Rzmp = NaoRobot.getWalkParameter(COP_Noise) * NaoRobot.getWalkParameter(COP_Noise);
    Rcom = NaoRobot.getWalkParameter(CoM_Noise) * NaoRobot.getWalkParameter(CoM_Noise);


    cout<<"VRP Delayed Kalman Filter Initialized Successfully"<<endl;
    firstrun = true;

}

void DelayedKalmanDCM::setInitialState(Vector4f x_)
{
    x = x_;
    firstrun = false;
}


void DelayedKalmanDCM::predict(float u)
{
    
    x = A*x + B*u;
    P = A*P*A.transpose();
    P.noalias() += Q;
    updateVars();
}


void DelayedKalmanDCM::updateWithCOP(float y)
{
    
    if(kbuffer.size() > (int) ZMPDELAY - 1){
        
        
        y -= Czmp*xbuffer.front();
        
        
        M = I;
        for (unsigned int j =0; j<(int) ZMPDELAY;j++)
        {
            M *= (I-kbuffer[j]*Czmp)*A;
        }
        
        
        s = Rzmp;
        s += Czmp * pbuffer.front() * Czmp.transpose();
        K = M*pbuffer.front() * Czmp.transpose() / s;
        x += K*y;
        //Update the error covariance
        P -= K*Czmp*pbuffer.front()*M.transpose();
        
        pbuffer.pop();
        xbuffer.pop();
        kbuffer.pop_front();
        kbuffer.push_back(K);
        
    }
    else
        kbuffer.push_back(Matrix4_1f::Zero());
    

    pbuffer.push(P);
    
    xbuffer.push(x);
    
    
    updateVars();
    
    
}
Vector4f DelayedKalmanDCM::getState()
{
    return x;
}

void DelayedKalmanDCM::updateWithCoM(float y)
{
    
    y -= Ccom*x;
    s = Rcom;
    s += Ccom * P * Ccom.transpose();
    K = P * Ccom.transpose() / s;
    x += K*y;
    //Update the error covariance
    P = (I - K * Ccom) * P * (I - K * Ccom).transpose();
    P.noalias() +=  K * Rcom * K.transpose();
    updateVars();

}

void DelayedKalmanDCM::updateVars()
{
    com = x(0);
    dcm = x(1);
    vrp = x(2);
    dist = x(3);
}
