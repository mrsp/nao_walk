#ifndef __MPCDCM_H__
#define __MPCDCM_H__
#include "RobotParameters.h"
#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include "KMat.hpp"
#include <iostream>
#include "DelayedObserverDCM.h"
#define Np 122
using namespace Eigen;
using namespace std;
class MPCDCM
{
    private:
        
        
        RobotParameters &NaoRobot;



        float    du_x, du_y, qx, qv, u_x, u_y;
        Vector2f DCM_, VRP_;
 	    Vector4f x,y,x_,y_, xOUT, yOUT;
        Vector3f xe, ye;
        MatrixXf Fx, Fv, Fxu, Fvu, R, Qx, Qv, K_X, K_V,H , tmpb, H_inv, Ad, Ae, Cd, A, L;
        VectorXf K_v, VRPRefX, VRPRefY, Cksi,  Ce, Cx, Be, Bd, temp, K_x, B, C;
        
        
        
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        float vrpx_d,comx_d,vrpy_, vrpx_,comdx_d,dcmx_d,dcmdx_d, vrpy_d,comy_d,comdy_d,dcmy_d,dcmdy_d,  comddx_d, comddy_d;
        bool firstrun;
        DelayedObserverDCM dObsDCMx, dObsDCMy;
        MPCDCM(RobotParameters &robot);
        
        void setInitialState(Vector2f DCM, Vector2f CoM, Vector2f ZMP);
        
        void Control(boost::circular_buffer<KVecFloat3> & VRPRef, Vector2f DCM, Vector2f CoM, Vector2f ZMP);
        
        
        
};
#endif
