#ifndef  __DELAYEDOBSERVERDCM_H__
#define  __DELAYEDOBSERVERDCM_H__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "RobotParameters.h"
#include <queue>

#define ZMPDELAY 18
using namespace Eigen;
using namespace std;

typedef Matrix<float,4,1> Matrix4_1f;
typedef Matrix<float,4,3> Matrix4_3f;
typedef Matrix<float,4,2> Matrix4_2f;
typedef Matrix<float,1,4> Matrix1_4f;
class DelayedObserverDCM
{
    
private:
    RobotParameters &NaoRobot;
    Matrix4f  A, I;
    Matrix1_4f Ccom, Czmp, Cdcm;
    Matrix4_1f B;
    Matrix4_2f Lcom;
    Matrix4_3f L;
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
    float Observer_COP, Observer_CoM;

    bool firstrun;
    void setInitialState(Vector4f x_);
    void update(float u_, float zmp_, float dcm_, float com_);
    DelayedObserverDCM(RobotParameters &robot);
    float com, vrp, dcm, dist;
    Vector4f getState();
};
#endif
