#ifndef  __TRAJPLAN_H__
#define  __TRAJPLAN_H__

#include "KMat.hpp"
#include "KWalkMat.h"
#include "MotionDefines.h"
#include "RobotParameters.h"
#include "robot_consts.h"
#include <queue>          // std::queue
#include <boost/circular_buffer.hpp>
#include <iostream>
#include "StepAdjustment.h"

using namespace std;
class TrajectoryPlanner
{
    
private:
    RobotParameters &NaoRobot;
    WalkInstruction planned, zmpi, ci;
    KWalkMat interp;
    float maxX,maxY,minX,minY;
public:
    /** Walking Instruction to be executed **/
    
    std::queue<WalkInstruction> walkInstructionbuffer;
    KVecFloat3 target, start, startL, startR, targetR, targetL, plantargetL, plantargetR, plantarget, planstartL, planstartR;
    /** ZMP Buffer for the Prediction Horizon **/
    boost::circular_buffer<KVecFloat3> ZMPbuffer;
    /** Queues needed for planning **/
    std::queue<WalkInstruction> stepAnkleQ, zmpQ;
    bool planAvailable, firstrun;

    //Methods
    void planZMPTrajectory();
    void computeSwingLegAndZMP(KVecFloat3 &tl, KVecFloat3 &tr, KVecFloat3 &t, KVecFloat3 sl,  KVecFloat3 sr, WalkInstruction i);
    void generatePlan(KVecFloat2 DCM_, KVecFloat2 COP_,bool UseStepAdjustment);
    void init(KVecFloat3 sl, KVecFloat3 sr);
    void emptyPlan();
    void plan(KVecFloat2 DCM_,KVecFloat2 COP_,bool UseStepAdjustment);
    TrajectoryPlanner(RobotParameters &robot);
    //Step Adjustment
    StepAdjustment sa;
    KVecFloat2 dx, tempV;
    KMath::KMat::GenMatrix<float,2,2>  SFoot_rot;
    float SFoot_angle;
    
    void computeFeetAnkleFromFeetCenter(KVecFloat3& al, KVecFloat3& ar, KVecFloat3 cl, KVecFloat3 cr);
    void computeFeetCenterFromFeetAnkle(KVecFloat3& cl, KVecFloat3& cr, KVecFloat3 tl, KVecFloat3 tr);
    WalkInstruction getCurrWalkInstruction();
    void setInitialFeet(KVecFloat3 sl, KVecFloat3 sr);
    /** @fn float anglemean(float l, float r) const
     *  @brief Computation of mean angle
    **/
   
    float anglemean(float l, float r) const
    {
        return (float) ((double) l + KMath::anglediff2( (double) r, (double) l) / 2.0);
    }
    float cropStep(float f_, float max_, float min_)
    {
        return fmax(min_, fmin(f_, max_));
    }
};
#endif
