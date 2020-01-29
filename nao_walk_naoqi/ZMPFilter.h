#ifndef  __ZMPFILTER_H_
#define  __ZMPFILTER_H_
#include <eigen3/Eigen/Dense>
#include "RobotParameters.h"
#include <iostream>
using namespace Eigen;
using namespace std;

class ZMPFilter
{
    
private:
    float alpha, gravity, m;
    Matrix3f Ib;
    RobotParameters NaoRobot;
public:
    void reset();
    
    /** @fn void filter(Vector3f  cop, Vector3f CoM, Vector3f Acc, Vector3f Gyro, Vector3f Gyrodot)
     *  @brief filters the  measurement with an average filter
     */
    ZMPFilter(RobotParameters &robot);
    Vector3f filter(Vector3f  cop, Vector3f CoM, Vector3f Acc, Vector3f Gyro, Vector3f Gyrodot, Matrix3f Rotwb);
};
#endif