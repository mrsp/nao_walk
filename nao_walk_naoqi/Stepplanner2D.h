/*! \file StepPlanner.h
 *	\brief A Monas Activity that first plans the trajectories needed by the Walk Engine
 and executes the desired walking gait!
 *
 */

#ifndef _STEPPLANNER2D_H
#define _STEPPLANNER2D_H

#include "RobotParameters.h"
#include "MotionDefines.h"
#include <boost/circular_buffer.hpp>

using namespace std;

class Stepplanner2D 
{
    private:
        KVecFloat3 v_;
        KVecFloat2 targetXY, pivotXY, centerXY, c, h, tempV, MaxStep, MinStep;
        KMath::KMat::GenMatrix<float,2,2> rot;
        int cmd, step_id;
        float eps, support_foot_x, support_foot_y, support_foot_orientation, targetTheta;
    RobotParameters robot;
    public:
    Stepplanner2D(RobotParameters robot_);
    WalkInstruction planStep2D(KVecFloat3 v, WalkInstruction si);
    bool planAvailable;
    void emptyPlan();
    void plan(WalkInstruction si);
    boost::circular_buffer<WalkInstruction> stepAnkleQ; 
    boost::circular_buffer<KVecFloat3> velocityQ; 
    float cropStep(float f_, float max_, float min_)
    {
        return max(min_, min(f_, max_));
    }
};
#endif

