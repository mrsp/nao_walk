#ifndef  __DELAYEDKALMANDCM_H__
#define  __DELAYEDKALMANDCM_H__

#include <eigen3/Eigen/Dense>
#include <queue>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include "RobotParameters.h"
#define ZMPDELAY 8
using namespace Eigen;
using namespace std;

typedef Matrix<float,4,1> Matrix4_1f;
typedef Matrix<float,1,4> Matrix1_4f;
class DelayedKalmanDCM
{
    
private:
    RobotParameters &NaoRobot;
    Matrix4f P, Q, A, I, M;
    Matrix1_4f Ccom, Czmp;
    Matrix4_1f K, B;
    Vector4f x;
    float Rcom, Rzmp, s;
    std::queue<VectorXf> xbuffer;
    std::queue<MatrixXf> pbuffer;
    boost::circular_buffer<MatrixXf> kbuffer;
    void updateVars();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** @fn void Filter(float ZMPMeasured,float CoMMeasured)
     *  @brief filters the ZMP measurement from the CoP using
     *  also the COM measured by the encoders
     */
    bool firstrun;
    void setInitialState(Vector4f x_);
    void predict(float u_);
    void updateWithCOP(float zmp);
    void updateWithCoM(float com);
    DelayedKalmanDCM(RobotParameters &robot);
    Vector4f getState();
    float CoM_Noise, COP_Noise;
    float com, vrp, dcm, dist;
};
#endif
