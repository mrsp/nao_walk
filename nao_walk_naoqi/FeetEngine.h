#ifndef  __FEETENGINE_H__
#define  __FEETENGINE_H__
#include "KMat.hpp"
#include "Common.hpp"
#include "KWalkMat.h"
#include "RobotParameters.h"



class FeetEngine
{

private:

    RobotParameters &NaoRobot;
    KWalkMat interp;


public:
    KMath::KMat::GenMatrix<float, 3, 1> startL, planL, startR, planR;

    //Current Desired FootHold position xytheta
    KMath::KMat::GenMatrix<float, 3, 1> FootL, FootR;
    float startLz,startRz, planRz, planLz, FootLz, FootRz, StepZ_;

    FeetEngine(RobotParameters &robot);
    void reset(); 
    void default_params();
    void setFootStartXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight);
    void setFootDestXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight);
    void setFootXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight);
    void setFootStartZ(float z, bool isRight);
    void setFootDestZ(float z, bool isRight);
    void setFootZ(float z, bool isRight);
    KVecFloat3 getFootDestXYTheta(bool isRight);
    float getFootDestTheta(bool isRight);
    float getFootDestZ(bool isRight);
    KVecFloat3 getFootXYTheta(bool isRight);
    float getFootTheta(bool isRight);
    float getFootZ(bool isRight);
    void MotionPlan(KVecFloat3 target, unsigned step, unsigned totalsteps, bool right_support, bool double_support, bool RightEarlyContact, bool LeftEarlyContact, bool RightLateContact, bool LeftLateContact);
};
#endif
