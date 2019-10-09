#include "PostureController.h"
#include "KinematicsDefines.h"
#include "ZMPDistributor.h"

PostureController::PostureController(RobotParameters &robot): NaoRobot(robot)
{
    dTorso_Roll = 0.0;
    dTorso_Pitch = 0.0;

    dLAnkle_Roll = 0.0;
    dLAnkle_Pitch = 0.0;
    dRAnkle_Roll = 0.0;
    dRAnkle_Pitch = 0.0;

    dKnee_Pitch = 0.0;
    TH = 0.0 * 3.14/180.0;
    default_params();
    std::cout<<"Real-Time Posture Controller Initialized Successfully"<<std::endl;
}



void PostureController::default_params()
{
    /* For Angular Momentum Control in Pattern Generator */
    amX = 1.0;
    amY = 1.0;
    
    dt = NaoRobot.getWalkParameter(Ts);
    
    Tc_ = NaoRobot.getWalkParameter(Tc);
    Kc_ = NaoRobot.getWalkParameter(Kc);

    Ta_ = NaoRobot.getWalkParameter(Ta);
    Ka_ = NaoRobot.getWalkParameter(Ka);

    Tn_ = NaoRobot.getWalkParameter(Tn);
    Kn_ = NaoRobot.getWalkParameter(Kn);
}



KVecFloat2 PostureController::soleAnguralMomentumStabilizer(float Torso_Roll,float comX, float Torso_Pitch,float comY)
{
    
    KVecFloat2 CoM;
    
    float K_amX=amX*NaoRobot.getWalkParameter(Ts);
    float K_amY=amY*NaoRobot.getWalkParameter(Ts);
    CoM(0)=comX-K_amX*Torso_Pitch;
    CoM(1)=comY-K_amY*Torso_Roll;
    
    return CoM;
    
}



void PostureController::ankleStabilizer(Vector3f tauld, Vector3f taurd, Vector3f taul, Vector3f taur,
     float lankle_Pitch_d, float rankle_Pitch_d,float lankle_Roll_d,float rankle_Roll_d)
{
    
    
    Ta_ = NaoRobot.getWalkParameter(Ta);
    Ka_ = NaoRobot.getWalkParameter(Ka);
    dt = NaoRobot.getWalkParameter(Ts);





    dLAnkle_Roll = Ka_ * dt * (tauld(0) - taul(0)) + (1.0 - dt/Ta_) * dLAnkle_Roll;
    dLAnkle_Pitch = Ka_ * dt * (tauld(1) - taul(1)) + (1.0 - dt/Ta_) * dLAnkle_Pitch;

  
    dRAnkle_Roll = Ka_ * dt * (taurd(0) - taur(0)) + (1.0 - dt/Ta_) * dRAnkle_Roll;
    dRAnkle_Pitch = Ka_ * dt * (taurd(1) - taur(1)) + (1.0 - dt/Ta_) * dRAnkle_Pitch;




    //Ankle Roll
    lankle_Roll=lankle_Roll_d - dLAnkle_Roll;
    rankle_Roll=rankle_Roll_d - dRAnkle_Roll;

    //Ankle Pitch
    lankle_Pitch=lankle_Pitch_d - dLAnkle_Pitch;
    rankle_Pitch=rankle_Pitch_d - dRAnkle_Pitch;


    //Bounds Checking
    //Ankle Joint
    if(lankle_Pitch>LAnklePitchHigh)lankle_Pitch=LAnklePitchHigh;
    if(lankle_Pitch<LAnklePitchLow) lankle_Pitch=LAnklePitchLow;
    if(rankle_Pitch>RAnklePitchHigh) rankle_Pitch = RAnklePitchHigh;
    if(rankle_Pitch<RAnklePitchLow) rankle_Pitch = RAnklePitchLow;
    
    if(lankle_Roll>LAnkleRollHigh) lankle_Roll=LAnkleRollHigh;
    if(lankle_Roll<LAnkleRollLow) lankle_Roll=LAnkleRollLow;
    if(rankle_Roll>RAnkleRollHigh) rankle_Roll=LAnkleRollHigh;
    if(rankle_Roll<RAnkleRollLow) rankle_Roll=LAnkleRollLow;
}

void PostureController::kneeStabilizer(float flz, float frz,  float  flz_d, float  frz_d, float lknee_Pitch_d, float rknee_Pitch_d)
{
    Tn_ = NaoRobot.getWalkParameter(Tn);
    Kn_ = NaoRobot.getWalkParameter(Kn);
    dt = NaoRobot.getWalkParameter(Ts);
    float deltaF = flz - frz;
    float deltaF_d = flz_d - frz_d;

    dKnee_Pitch = Kn_ * dt * (deltaF_d - deltaF) + (1.0 - dt/Tn_) * dKnee_Pitch;


    lknee_Pitch = lknee_Pitch_d - dKnee_Pitch;
    rknee_Pitch = rknee_Pitch_d + dKnee_Pitch;
    if(lknee_Pitch>LKneePitchHigh)lknee_Pitch=LKneePitchHigh;
    if(lknee_Pitch<LKneePitchLow) lknee_Pitch=LKneePitchLow;
    if(rknee_Pitch>RKneePitchHigh)rknee_Pitch=RKneePitchHigh;
    if(rknee_Pitch<RKneePitchLow) rknee_Pitch=RKneePitchLow;
}
                                            

void PostureController::torsoStabilizer(float Torso_Roll, float Torso_Pitch, float Torso_Roll_d, float Torso_Pitch_d, float lhip_Pitch_d, float rhip_Pitch_d,float lhip_Roll_d,float rhip_Roll_d)
{
    
    Tc_ = NaoRobot.getWalkParameter(Tc);
    Kc_ = NaoRobot.getWalkParameter(Kc);
  
    dt = NaoRobot.getWalkParameter(Ts);
    dTorso_Roll = Kc_ * dt * (Torso_Roll_d - Torso_Roll) + (1.0 - dt/Tc_) * dTorso_Roll;
    dTorso_Pitch = Kc_ * dt * (Torso_Pitch_d - Torso_Pitch) + (1.0 - dt/Tc_) * dTorso_Pitch;


    lhip_Roll = lhip_Roll_d  - dTorso_Roll;
    rhip_Roll = rhip_Roll_d  - dTorso_Roll;
    lhip_Pitch = lhip_Pitch_d - dTorso_Pitch;
    rhip_Pitch = rhip_Pitch_d - dTorso_Pitch;
    

  
    //Bounds Checking
    //Hip Pitch Joint
    if(lhip_Pitch>LHipPitchHigh - TH)lhip_Pitch=LHipPitchHigh - TH;
    if(lhip_Pitch<LHipPitchLow + TH) lhip_Pitch=LHipPitchLow + TH;
    if(rhip_Pitch>RHipPitchHigh - TH) rhip_Pitch = RHipPitchHigh - TH;
    if(rhip_Pitch<RHipPitchLow + TH) rhip_Pitch = RHipPitchLow + TH;
    //Hip Roll Joint
    if(lhip_Roll>LAnkleRollHigh - TH) lhip_Roll=LHipRollHigh - TH;
    if(lhip_Roll<LAnkleRollLow + TH) lhip_Roll=LHipRollLow + TH;
    if(rhip_Roll>RAnkleRollHigh - TH) rhip_Roll=LHipRollHigh - TH;
    if(rhip_Roll<RAnkleRollLow + TH) rhip_Roll=LHipRollLow + TH;
}
