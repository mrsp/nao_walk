#ifndef  __DELAYEDOBSERVERDCM_H__
#define  __DELAYEDOBSERVERDCM_H__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "RobotParameters.h"
#include <queue>

#define ZMPDELAY 3
using namespace Eigen;
using namespace std;

typedef Matrix<float,4,1> Matrix4_1f;
typedef Matrix<float,4,2> Matrix4_2f;
typedef Matrix<float,1,4> Matrix1_4f;
class DelayedObserverDCM
{
    
private:
    RobotParameters &NaoRobot;
    Matrix4f  A, I;
    Matrix1_4f Ccom, Czmp;
    Matrix4_1f B, Lcom;
    Matrix4_2f L;
    //std::queue<KVecFloat4> xbuffer;
    std::queue<VectorXf> xbuffer;

    Vector4f x;
    void updateVars();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** @fn void Filter(float ZMPMeasured,float CoMMeasured)
     *  @brief filters the ZMP measurement from the CoP using
     *  also the COM measured by the encoders
     */
    bool firstrun;
    void setInitialState(Vector4f x_);
    void update(float u_, float com_, float zmp_);
    DelayedObserverDCM(RobotParameters &robot);
    float com, vrp, dcm, dist;
    Vector4f getState();
};
#endif
