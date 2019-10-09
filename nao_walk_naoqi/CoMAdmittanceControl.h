#ifndef __COMADMITTANCECONTROL__
#define __COMADMITTANCECONTROL__
#include "RobotParameters.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "LeakyIntegrator.h"
#include <queue>
using namespace Eigen;
using namespace std;

class CoMAdmittanceControl
{
private:
    
    RobotParameters NaoRobot;
    LeakyIntegrator ZMPXint, ZMPPXint,ZMPYint, ZMPPYint;
    Matrix2d k_f;
    Vector2d int_zmp, int_zmpp, zmp_out, zmp_d_;
    double rateX, rateY;
    bool firstrun;
    std::queue<VectorXd> zmp_d_buffer;
    int zmp_delay;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double gain_x, gain_y;
    double K_pitch, K_roll;

    CoMAdmittanceControl(RobotParameters &robot);

    
    Vector3d com_c;

    Vector3d Control(Vector2d com_d,Vector2d comd_d,Vector2d vrp_d,Vector2d vrp,Vector2d com);
    Vector3d Control(Vector2d com_d,Vector2d zmp_d, Vector2d zmp, double roll, double pitch, double omega_roll, double omega_pitch, bool GroundContact);
    Vector3d ControlNoDelay(Vector2d com_d, Vector2d zmp_d, Vector2d zmp, double roll, double pitch, double omega_roll, double omega_pitch, bool GroundContact);
};


#endif
