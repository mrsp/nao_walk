#ifndef __POSTURECONTROLLER_H__
#define __POSTURECONTROLLER_H__
#include "RobotParameters.h"
#include "KMat.hpp"
#include <eigen3/Eigen/Dense>


using namespace Eigen;

class PostureController
{
private:
    
    RobotParameters NaoRobot;
    
    float TH;
    void default_params();
public:
    /** Ankle PD Control **/
    float Kcx_, Kcy_, Tcx_, Tcy_, Ka_, Ta_, Kn_, Tn_, dt;
    float dTorso_Roll, dTorso_Pitch, dLAnkle_Roll, dLAnkle_Pitch, dRAnkle_Roll, dRAnkle_Pitch, dKnee_Pitch;
    /** AM Control **/
    float amX, amY;
    
    float lankle_Pitch, rankle_Pitch, lankle_Roll, rankle_Roll;
    float lhip_Pitch, rhip_Pitch, lhip_Roll, rhip_Roll;
    float lknee_Pitch, rknee_Pitch;

    PostureController(RobotParameters &robot);
    
    
    KVecFloat2 soleAnguralMomentumStabilizer(float Torso_Roll,float comX, float Torso_Pitch,float comY);
    
    void kneeStabilizer(float flz, float frz,  float  flz_d, float  frz_d,
                                            float lknee_Pitch_d, float rknee_Pitch_d);
    void ankleStabilizer(Vector3f tauld, Vector3f taurd, Vector3f taul, Vector3f taur,
                                    float lankle_Pitch_d, float rankle_Pitch_d,float lankle_Roll_d,float rankle_Roll_d);
    void torsoStabilizer(float Torso_Roll, float Torso_Pitch, float Torso_Roll_d, float Torso_Pitch_d, 
                            float lhip_Pitch_d, float rhip_Pitch_d,float lhip_Roll_d,float rhip_Roll_d);
};
#endif
