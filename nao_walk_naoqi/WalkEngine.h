#ifndef __WALKENGINE_H__
#define __WALKENGINE_H__

#include "KMat.hpp"
#include "robot_consts.h"
#include "KinematicsDefines.h"
#include "NAOKinematics.h"
#include "RobotParameters.h"
#include "KWalkMat.h"
#include "Common.hpp"
#include "CoMEKF.h"
#include "PCThread.h"
#include "PostureController.h"
#include "MPCDCM.h"
#include "CoMAdmittanceControl.h"
#include "matlog.h"
//#include "JointSSKF.h"
#include "FeetEngine.h"
#include "MotionDefines.h"
#include "butterworthLPF.h"
#include "butterworthHPF.h"
#include "TrajectoryPlanner.h"
#include <eigen3/Eigen/Dense>
#include "ZMPDistributor.h"
#include "Stepplanner2D.h"
//#include "LIPMThread.h"
#include "ZMPFilter.h"
#include<list>
using namespace Eigen;


using namespace std;
class WalkEngine
{
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    WalkInstruction executedStep;
    bool _isFootStepExecuted, isEstimationLoopInitialized;
    int stepPhaseId;

    /** WalkEngine Constructor **/
    WalkEngine(RobotParameters &rp);
    
    bool isFootStepExecuted(int id)
    {
        return last_step_id>=id;
    }
    
    /** Necessary Instances by WalkEngine **/
    RobotParameters &NaoRobot;
    
    KWalkMat interp;
    ZMPFilter zmpFilter;
    FeetEngine NaoFeetEngine;
    
    NAOKinematics nkin;
    
    TrajectoryPlanner tp;        
    /** MPC Controller **/
    PCThread NaoLIPM;
    //LIPMThread NaoLIPM;

    CoMAdmittanceControl NaoVRPToCoM;
    MPCDCM NaoMPCDCM;

    //mat_log flog;
    ZMPDistributor zmpDist;

    Affine3f Tis_eig, Tib_eig, Tbs_eig, Tssprime_eig, Til_eig, Tir_eig;
    
    
    butterworthLPF* leftGRF_LPF;
    butterworthLPF* rightGRF_LPF;
    Vector3f copi,copi_cropped;
    
    
    //JointSSKF JointKF[KDeviceLists::NUMOFJOINTS];
    
    CoMEKF* nipmEKF;
    
    /** Posture Controller **/
    PostureController NaoPosture;
    
    Vector3f forceL, forceR, forceLw, forceRw;
    
    Stepplanner2D sp;

    butterworthLPF *coplx_LPF,*coply_LPF,*coprx_LPF,*copry_LPF;
    butterworthLPF *AccX_LPF, *AccY_LPF, *AccZ_LPF, *GyroX_LPF, *GyroY_LPF;
    butterworthHPF *AccX_HPF, *AccY_HPF, *AccZ_HPF, *GyroX_HPF, *GyroY_HPF;
    int startup, transitionSI_idx;
    bool noContactLeft, noContactRight;
    
    bool  GroundContact, inAir, executedAStep;
    
    bool legEarlyLand, isEngineInitialized;
    
    /** Arm joint data **/
    Matrix<float, 8, 1> armangles,armangles_s, armangles_e;
    Vector4f armangles_temp;
    std::vector<float> alljoints;
    
    /** WalkEngine Public functions **/
    

    WalkInstruction getCurrStep();



    /** @fn std::vector<float> runStep()
     *  @brief Main Walking Engine function
     *   computes the desired trajectories and
     *   the joints
     **/
    
    std::vector<float> runStep();
    /** @fn void reset()
     *  @brief resets the Walking Engine
     **/
    
    void reset();
    /** @fn void addInit()
     *  @brief initializes the Walking Engine
     **/
    void addWalkInstruction();
    void AddVelocityInstruction(KVecFloat3 vel);
    void addWalkInstruction(WalkInstruction &i);
    
    /** @fn void setFSR(KMath::KMat::GenMatrix<float, 4, 1> l, KMath::KMat::GenMatrix<float, 4, 1> r)
     *  @brief Sets the values of the FSRs
     **/
    void setFSR(Vector4f l,Vector4f r, Vector3f cl, Vector3f cr, float wl, float wr );
    
    /** @fn void initFSR(KMath::KMat::GenMatrix<float, 3, 4> l, KMath::KMat::GenMatrix<float, 3, 4> r)
     *  @brief initializes the FSR posistions
     **/
    void initFSR(Matrix<float, 3, 4> l, Matrix<float, 3, 4> r)
    {
        
        fsrposl = l;
        fsrposr = r;
        isFSRInitialized = true;
    };
    
    
    void setIMU(Vector3f acc, Vector3f gyro,  Vector3f angle);
    
    void setJoints(std::vector<float> joints);
    
    void resetBuffers();
    
    int assignStepID();
    int step_id;
    //void computeKinTFs();
    //void initializeTFs();
    
    
    
    Vector3f odomTrans;
    Quaternionf odomQuat;
    
    bool changeContact;
    /**  support Indicators **/
    bool right_support_id, double_support_id;
    bool isFSRInitialized;
    
    
    int cstep_id; //current step id
    int last_step_id; //current step id
private:
    
    void setCurrStep(const WalkInstruction &i);
    boost::circular_buffer<WalkInstruction> stepAnkleQ; 
    boost::circular_buffer<KVecFloat3> velocityQ; 

    bool FSRDataInit, JointDataInit, IMUDataInit;
    bool firstGyrodot;
    Matrix3f Inertia;
    /** IMU DATA **/
    Vector3f  Angle,Gyro, Acc, Gyro_, Gyrodot;
    /** CoM by encoders in Meters **/
    Vector3f CoMm;
    
    /** Result of Inverse Kinematics **/
    std::vector<float> ret, leg_joint_targets, leg_joint_targets_start, leg_joint_targets_end;
    /** Joint Vectors obtained from the Inverse Kinematics **/
    std::vector<std::vector<float> > resultR, resultL;
    
    /** Early Landing Indicators **/
    bool  rightEarlyContact, leftEarlyContact, leftLateContact, rightLateContact;
    
    
    /** FSR position matrices and FSR Data vectors **/
    Matrix<float, 3, 4> fsrposl, fsrposr;
    Vector4f fsrl, fsrr;
    Vector3f copl, copr, copl_cropped, copr_cropped;
    float weightl, weightr;
    int ci_id;
    bool postureInitialized;
    Vector3f AccW, GyroW;
    /** Real odometry Data **/
    //Odometry, from supportleg to inertial, transformation from support leg to other leg
    NAOKinematics::kmatTable Tip,Tis, Tis_, Tip_, Tsp, Tps, Tssprime, Til, Tir;
    
    
    float support_foot_, double_support_; //1 for left 0 for right;
    Vector3f copsrel, fis;
    Vector2d CoM_c;
    
    
    /** The Inverse Kinematics Targets **/
    NAOKinematics::kmatTable Tpprimel,Tpprimer;
    
    /**
     *  target  points at the end of cycle (XYTheta)
     * **/
    KVecFloat3  planL, planR;
    
    /** Support Leg **/
    KDeviceLists::SupportLeg supportleg, oldsupportleg;
    
    KDeviceLists::ChainsNames chainsupport, chainswing;
    
    
    /**
     * Current Walking Instruction at the walking cycle
     * Planned Walking Instruction at the end of the walking cycle
     **/
    WalkInstruction ci;
    SupportInstruction si;
    /** Current Walking Command counter **/
    unsigned currentstep;
    
    
    
    /** Measured Center of Mass from the joint encoders **/
    KVecDouble3 CoMs, CoMi;
    /** All in inertial frame **/
    KVecDouble3 com_error,desired;
    
    void updateTFs();
    void stateEstimationLoop();

    void determineLegContact();
    /** @fn void feed()
     *  @brief Filling the Command and ZMP buffers
     **/
    void feed();
    
    /** @fn void Calculate_Desired_COM()
     *  @brief Computation of the Target Center of Mass wrt the Inertial Frame of Reference
     **/
    void Calculate_Desired_COM();
    
    /** @fn std::vector<float> Calculate_IK()
     *  @brief Computation of leg joint angles
     **/
    std::vector<float> Calculate_IK();
    std::vector<float> Calculate_IK0();

    void computeGyrodot();
    /** @fn KVecFloat2 getCoP()
     *  @brief Computation of Center of Pressure, x,y data
     **/
    

    float crop(float v, float max_v, float min_v);
    Vector3f getCoP();
    
    /** @fn KVecFloat3 getPositionInertial(NAOKinematics::Effectors ef)
     *  @brief Compute the Position x, y and orientation omega
     *   wrt Inertial Frame of Reference
     **/
    inline KVecFloat3 getPositionInertial(NAOKinematics::Effectors ef)
    {
        KVecFloat3 t, r;
        /** Do not change the parenthesis **/
        NAOKinematics::kmatTable m = Tip*nkin.getForwardEffector(ef);
        t = m.getTranslation();
        r(0) = t(0) / 1000.0;
        r(1) = t(1) / 1000.0;
        r(2) = m.getEulerAngles()(2);
        return r;
        
    };
    
    inline float  getFootZInertial(NAOKinematics::Effectors ef)
    {
        KVecFloat3 t;
        float r;
        /** Do not change the parenthesis **/
        NAOKinematics::kmatTable m = Tip*nkin.getForwardEffector(ef);
        t = m.getTranslation();
        
        r = t(2) / 1000.0;
        return r;
        
    };
    
    /** @fn NAOKinematics::kmatTable getTransformation(KVecFloat3 p, float z)
     *  @brief Compute a Transformation matrix without the pitch roll angles
     **/
    inline NAOKinematics::kmatTable getTransformation(KVecFloat3 p, float z)
    {
        NAOKinematics::kmatTable t;
        KMath::KMat::transformations::makeTransformation(t,
                                                         (double) p(0) * 1000,
                                                         (double) p(1) * 1000,
                                                         (double) z * 1000,
                                                         0.0,
                                                         0.0,
                                                         (double) p(2)
                                                         );
        return t;
    };
    
    /** @fn float anglemean(float l, float r) const
     *  @brief Computation of mean angle
     **/
    
    inline float anglemean(float l, float r) const
    {
        return (float) ((double) l + KMath::anglediff2( (double) r, (double) l) / 2.0);
    }
    
    
    Affine3f transformKMatToEigen(NAOKinematics::kmatTable T)
    {
        Affine3f T_;
        T_.setIdentity();
        T_(0,0) = (float) T(0,0);
        T_(0,1) = (float) T(0,1);
        T_(0,2) = (float) T(0,2);
        T_(0,3) = (float) T(0,3)*0.001;
        
        T_(1,0) = (float) T(1,0);
        T_(1,1) = (float) T(1,1);
        T_(1,2) = (float) T(1,2);
        T_(1,3) = (float) T(1,3)*0.001;
        
        
        T_(2,0) =  (float) T(2,0);
        T_(2,1) =  (float) T(2,1);
        T_(2,2) =  (float) T(2,2);
        T_(2,3) =  (float) T(2,3)*0.001;
        
        
        return T_;
    }
    
    NAOKinematics::kmatTable transformEigenToKMat( Affine3f T)
    {
        NAOKinematics::kmatTable T_;
        
        T_(0,0) = T(0,0);
        T_(0,1) = T(0,1);
        T_(0,2) = T(0,2);
        T_(0,3) = T(0,3)*1000.0;
        
        T_(1,0) = T(1,0);
        T_(1,1) = T(1,1);
        T_(1,2) = T(1,2);
        T_(1,3) = T(1,3)*1000.0;
        
        
        T_(2,0) = T(2,0);
        T_(2,1) = T(2,1);
        T_(2,2) = T(2,2);
        T_(2,3) = T(2,3)*1000.0;
        
        T_(3,0) = T(3,0);
        T_(3,1) = T(3,1);
        T_(3,2) = T(3,2);
        T_(3,3) = T(3,3);
        
        return T_;
    }
    
    
    
    bool isNear(NAOKinematics::kmatTable T1, NAOKinematics::kmatTable T2, float thres)
    {
        NAOKinematics::FKvars v1,v2;
        
        v1.p = T1.getTranslation();
        v2.p = T2.getTranslation();
        
        thres*=1000;
        
        float dist = sqrt((v1.p(0)-v2.p(0))*(v1.p(0)-v2.p(0)) + (v1.p(1)-v2.p(1))* (v1.p(1)-v2.p(1)) + (v1.p(2)-v2.p(2))* (v1.p(2)-v2.p(2)));
        if (dist < thres)
            return true;
        else
            return false;
    }
    
    
    
};
#endif
