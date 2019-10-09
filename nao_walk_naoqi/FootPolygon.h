#ifndef __FOOTPOLYGON_H__
#define __FOOTPOLYGON_H__

#include "RobotParameters.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
  

class FootPolygon
{

    private:
        RobotParameters robot;
        float stepXF, stepXH, stepYL, stepYR;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector3f footLF, footRF, footLH, footRH, footA;
        Matrix<float,4,2> polygon;
        int numNodes;
        FootPolygon(RobotParameters& robot_);
        void setPolygon(Vector3f footA_, Affine3f T);
        bool checkIn(float x_, float y_);  
        bool pnpoly(float x_, float y_);
        float getAngle(float x1, float y1, float x2, float y2);

};


#endif