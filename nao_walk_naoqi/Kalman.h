#ifndef  __KALMAN_H__
#define  __KALMAN_H__
#include "KMat.hpp"
#include "RobotParameters.h"
#include <queue>
#define ZMPKALMANDELAY 5
class Kalman
{
    
private:
    KMath::KMat::GenMatrix<float,2,1> Ckalman;
    KMath::KMat::GenMatrix<float,1,2> Kgain;
    KMath::KMat::GenMatrix<float,2,2> s,MeasurementNoise;
    KMath::KMat::GenMatrix<float,1,1> P, ProcessNoise,Bkalman;
    RobotParameters &OurRobot;
public:
    KMath::KMat::GenMatrix<float,1,1> StateKalman,StatePredict;
    std::queue<float> uBuffer,combuffer;
    KMath::KMat::GenMatrix<float,2,1> ykalman;
    
    /** @fn void Filter(float ZMPMeasured,float CoMMeasured)
     *  @brief filters the ZMP measurement from the CoP using
     *  also the COM measured by the encoders
     */
    void Filter(float ZMPMeasured,float CoMMeasured);
    void reset();
    Kalman(RobotParameters &robot);
    double CoM_Noise, COP_Noise;
};
#endif
