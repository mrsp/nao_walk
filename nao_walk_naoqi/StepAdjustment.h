
#ifndef __STEPADJUSTMENT__
#define __STEPADJUSTMENT__
#include "RobotParameters.h"
#include <iostream>
#include <string>
#include "QuadProg++.hh"
#include <limits>
#include "KMat.hpp"
using namespace std;


class StepAdjustment
{
    private:    
        RobotParameters &robot;
        quadprogpp::Matrix<double> G, CE, CI;
        quadprogpp::Vector<double> g0, ce0, ci0, x;
        KMath::KMat::GenMatrix<double,2,2> rot, rot_T;
        KVecDouble2 L_max, L_min, b_nom, L_nom, tempV;
        int n, m, p, axis;
        double dt,w, T_max, T_min, T_nom, tau_min, tau_max, tau_nom,  lp, bx_nom, by_nom, Lx_max, Ly_max, Lx_min, Ly_min;
        double a_0, a_1, a_2, a_3, a_4, a_5;
        double a_11, a_12, a_21, a_22;
        double kx, ky;
    public:
            
        float step_locationx, step_locationy, step_instructions, step_duration, step_bx, step_by;
        StepAdjustment(RobotParameters &robot_);
        void solve(double ksix_0,  double ksiy_0, double copx_0, double copy_0, double vrpx_0, double vrpy_0, double vrpx_ref, double vrpy_ref, double support_orientation, int RSS);
        void solve(double roll, double pitch, double comZ, double vrpx_ref, double vrpy_ref);
};
#endif
