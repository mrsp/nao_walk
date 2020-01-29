#ifndef __COMADMITTANCECONTROL__
#define __COMADMITTANCECONTROL__
#include "RobotParameters.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "LeakyIntegrator.h"
using namespace Eigen;
using namespace std;

class CoMAdmittanceControl
{
private:
    
    RobotParameters NaoRobot;
    LeakyIntegrator ZMPXint, ZMPPXint,ZMPYint, ZMPPYint;
    Matrix2d k_f;
    Vector2d int_zmp, int_zmpp;
    double rateX, rateY;
    bool firstrun;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double gain_x, gain_y;

    CoMAdmittanceControl(RobotParameters &robot);

    
    Vector2d com_c;
    Vector2d Control(Vector2d com_d,Vector2d comd_d,Vector2d vrp_d,Vector2d vrp,Vector2d com);
    
};


#endif
