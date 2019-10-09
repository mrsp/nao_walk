#ifndef  __ZMPDistributor_H__
#define  __ZMPDistributor_H__
#include <eigen3/Eigen/Dense>
#include "RobotParameters.h"
#include "FootPolygon.h"
#include <iostream>
using namespace Eigen;
using namespace std;



class ZMPDistributor
{

private:
    RobotParameters robot;
    float maxForceReadingL, maxForceReadingR;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ZMPDistributor(RobotParameters& robot_);
    FootPolygon LFoot, RFoot;
    Vector3f frd, fld, fl, fr, tauld, taurd, tau0, taul, taur;
    Vector2f pL, pR, pa;
    float a;
    void computeDistribution(Vector3f pzmp_d, Vector3f pzmp, Vector3f fl, Vector3f fr, Vector3f fext, Vector3f LfootA, Vector3f RfootA, Affine3f Til, Affine3f Tir, bool right_support, bool double_support);

    Vector2f computePointLineIntersection(Vector2f point_, Vector2f linepoint0_, Vector2f linepoint1_)
    {   
        Vector2f res;
        res.setZero();
        float dx = linepoint0_(0)-linepoint1_(0);
        float dy = linepoint0_(1)-linepoint1_(1);
        float mag = sqrt(dx*dx + dy*dy);
        dx /= mag;
        dy /= mag;
        // translate the point and get the dot product
        float lambda = (dx * (point_(0) - linepoint1_(0))) + (dy * (point_(1) - linepoint1_(1)));
        res(0) = (dx * lambda) + linepoint1_(0);
        res(1) = (dy * lambda) + linepoint1_(1);
        return res;
    }
};
#endif
