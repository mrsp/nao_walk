#ifndef  __JOINTSSKF_H__
#define  __JOINTSSKF_H__
#include "KMat.hpp"
#include "RobotParameters.h"
class JointSSKF
{

private:
    KMath::KMat::GenMatrix<float,2,1> K;
    KMath::KMat::GenMatrix<float,2,2> F;
    KMath::KMat::GenMatrix<float,2,1> x;
    RobotParameters NaoRobot;
    bool firstrun;
public:
    float JointPosition;
    float JointVelocity;

    /** @fn void Filter(float JointPosMeasurement);
     *  @brief filters the Joint Position using the measurement by the encoders
     */

    void Filter(float JointPosMeasurement);
    void reset();
    void initialize(RobotParameters &robot);

};
#endif
