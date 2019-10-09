#ifndef  __COPKF_H__
#define  __COPKF_H__

#include <eigen3/Eigen/Dense>
#include <queue>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include "RobotParameters.h"
#define ZMPDELAY_COPKF 5
using namespace Eigen;
using namespace std;

typedef Matrix<double,4,2> Matrix4_2d;
class COPKF
{
    
private:
    MatrixXd P, Q, A, I, M;
    MatrixXd R, s;
    MatrixXd C, CCoM;
    MatrixXd B, K;
    VectorXd x;
    VectorXd y,zmpd_;
    std::queue<VectorXd> xbuffer;
    std::queue<MatrixXd> pbuffer;
    boost::circular_buffer<MatrixXd> kbuffer;
    RobotParameters robot;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @fn void Filter(float ZMPMeasured,float CoMMeasured)
     *  @brief filters the ZMP measurement from the CoP using
     *  also the COM measured by the encoders
     */
    bool firstrun;
    void setInitialState(Vector4d x_);
    void predict(Vector2d zmpd);
    void updateWithCOP(Vector2d zmp);
    void updateWithCoM(Vector3d com, Vector3d Acc, Vector3d Gyro);

    COPKF(RobotParameters robot_);
    double CoM_Noise, COP_Noise;
    double zmpx, zmpy;
};
#endif
