#include "COPKF.h"

COPKF::COPKF(RobotParameters robot_):kbuffer((int) ZMPDELAY_COPKF), robot(robot_)
{
    C.resize(2,4);
    CCoM.resize(2,4);
    P.resize(4,4);
    Q.resize(4,4);
    A.resize(4,4);
    I.resize(4,4);
    M.resize(4,4);
    R.resize(2,2);
    s.resize(2,2);
    B.resize(4,2);
    K.resize(4,2);
    x.resize(4,1);
    y.resize(2,1);
    zmpd_.resize(2,1);

    s.setZero();
    R.setZero();
    x.setZero();
    zmpd_.setZero();

    y.setZero();
    P.setIdentity();
    I.setIdentity();
    P*=1e-4;
    Q.setZero();
    Q(0,0) = 1e-4;
    Q(1,1) = 1e-4;
    Q(2,2) = 1e-4;
    Q(3,3) = 1e-4;
    //Filter Dynamics
    A.setIdentity();
    //A(0,1) = 1.000;
    //A(2,3) = 1.000;
    B.setZero();
    B(0,0) = 1.000;
    B(2,1) = 1.000;
    C.setZero();
    CCoM.setZero();
    C(0,0) = 1.000;
    C(1,2) = 1.000;
    C(0,2) = 1.000;
    C(1,3) = 1.000;
    CCoM(0,0) = 1.000;
    CCoM(1,2) = 1.000;
    CoM_Noise = 0.25;
    COP_Noise = 0.15;
    cout<<"ZMP Delayed Kalman Filter Initialized Successfully"<<endl;
    firstrun = true;
}

void COPKF::setInitialState(Vector4d x_)
{
    x = x_;
    firstrun = false;

}


void COPKF::predict(Vector2d zmpd){

    x = A*x + B*(zmpd-zmpd_);
    P = A*P*A.transpose();
    P.noalias() += Q;
    zmpd_ = zmpd;


}


void COPKF::updateWithCOP(Vector2d zmp)
{

    if(kbuffer.size() > (int) ZMPDELAY_COPKF - 1){
 
        
        R(0,0) = COP_Noise * COP_Noise;
        R(1,1) = COP_Noise * COP_Noise;
        y = zmp;
        y.noalias() -= C*xbuffer.front();




        M = I;
        for (unsigned int j =0; j<(int) ZMPDELAY_COPKF;j++)
        {
            M *= (I-kbuffer[j]*C)*A;
        }
 

        s = R;
        s.noalias() += C * pbuffer.front() * C.transpose();
        K = M*pbuffer.front() * C.transpose() * s.inverse();
        x += K*y;
        //Update the error covariance
        P -= K*C*pbuffer.front()*M.transpose();

        pbuffer.pop();
        xbuffer.pop();
        kbuffer.pop_front();
        kbuffer.push_back(K);

    }
    else
        kbuffer.push_back(Matrix<double,4,2>::Zero());

    pbuffer.push(P);

    xbuffer.push(x);


    zmpx = x(0);
    zmpy = x(2);




}


void COPKF::updateWithCoM(Vector3d com, Vector3d Acc, Vector3d Gyro)
{
  	Acc += Gyro.cross(Gyro.cross(com));

    R(0,0) = CoM_Noise * CoM_Noise;
    R(1,1) = CoM_Noise * CoM_Noise;


    y = com.segment<2>(0);
    y.noalias() -=  Acc.segment<2>(0) * com(2)/(Acc(2) + robot.getWalkParameter(g));

    y.noalias() -= CCoM*x;

    s = R;
    s.noalias() += CCoM * P * CCoM.transpose();
    K = P * CCoM.transpose() * s.inverse();
    
    x += K*y;
    //Update the error covariance
    P = (I - K * CCoM) * P * (I - K * CCoM).transpose();
    P.noalias() +=  K * R * K.transpose();

}

