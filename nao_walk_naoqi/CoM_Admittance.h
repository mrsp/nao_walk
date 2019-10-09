#ifndef __CoM_Admittance_H__
#define __CoM_Admittance_H__
#include "KMat.hpp"
#include "RobotParameters.h"
class CoM_Admittance
{
private:
    
    
    RobotParameters &NaoRobot;
    
    float K, D, M, dt;
    
    KMath::KMat::GenMatrix<float, 2, 2> A;
    KMath::KMat::GenMatrix<float, 2, 1> B, e, temp;
    
public:
    
    bool resetted;
    
    
    CoM_Admittance(RobotParameters &robot);
    
    
    
    float Control(float force);
    
    
    void reset();
    
};
#endif
