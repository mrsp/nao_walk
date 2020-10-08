#ifndef __PCTHREAD_H__
#define __PCTHREAD_H__


#define PWindow 51


#include "KMat.hpp"
#include <iostream>
#include <boost/circular_buffer.hpp>
#include "RobotParameters.h"
#include <queue>
#define ZMPKALMANDELAY 3
using namespace std;
class Dynamics
{
private:

    KMath::KMat::GenMatrix<float, 4, 4> Ad;

    KMath::KMat::GenMatrix<float,4,1> Bd,temp;

    KMath::KMat::GenMatrix<float, 4, 2>  L;

    RobotParameters &OurRobot;


public:

    KMath::KMat::GenMatrix<float, 3, 1> Cd;

    KMath::KMat::GenMatrix<float,4,1> State;

    float zmpstate, zmpstate_;

    Dynamics(RobotParameters &robot);


    double Observer_COP, Observer_CoM;
    /** @fn Update(float u, KVecFloat2 error)
     *  @brief Update the Linear Dynamics
     * of the Cart and Table
    **/
    void Update(float u,KVecFloat2 error);


};



class Kalman
{

private:
    KMath::KMat::GenMatrix<float,2,1> Ckalman;
    KMath::KMat::GenMatrix<float,1,2> Kgain;
    KMath::KMat::GenMatrix<float,2,2> s,MeasurementNoise;
    KMath::KMat::GenMatrix<float,1,1> P, ProcessNoise,Bkalman;
    RobotParameters &OurRobot;
public:
    KMath::KMat::GenMatrix<float,1,1> StateKalman,StatePredict;
    std::queue<float> uBuffer,combuffer;
    KMath::KMat::GenMatrix<float,2,1> ykalman;

    /** @fn void Filter(float ZMPMeasured,float CoMMeasured)
     *  @brief filters the ZMP measurement from the CoP using
     *  also the COM measured by the encoders
     */
    void Filter(float ZMPMeasured,float CoMMeasured);
    void reset();
    Kalman(RobotParameters &robot);
    double CoM_Noise, COP_Noise;
};


class PCThread
{
private:


    KMath::KMat::GenMatrix<float, PWindow , 1> ZMPReferenceX, ZMPReferenceY;
    /** DMPC **/

    KMath::KMat::GenMatrix<float,1,3> Gx;

    KMath::KMat::GenMatrix<float, PWindow-1, 1> Gd;

    float Gi,Integrationfbx,Statefbx,Predictionfbx,Integrationfby,Statefby,Predictionfby,uX,uY;

public:

    RobotParameters &OurRobot;

	
    Dynamics DynamicsX, DynamicsY;
    Kalman KalmanX, KalmanY;
    PCThread(RobotParameters&);

    /** @fn void LIPMComPredictor(CircularBuffer<KVecFloat3> & ZmpBuffer, float CoMmeasuredX, float CoMmeasuredY, float ZMPMeasuredX, float ZMPMeasuredY);
     *  @brief Computes the desired COM
     */
    void setInitialState(KVecFloat2 CoM, KVecFloat2 ZMP);
    void Control(boost::circular_buffer<KVecFloat3> & ZmpBuffer, float CoMMeasuredX, float CoMMeasuredY, float CoMMeasuredZ, float ZMPMeasuredX, float ZMPMeasuredY);

    float vrpx_d, vrpy_d, comx_d, comy_d, comdx_d, comdy_d, zmpx, zmpy, dcmx_d, dcmy_d, vrpdx_d, vrpdy_d, zmpx_ref, zmpy_ref;
    float Observer_CoMX,Observer_CoMY,Observer_COPX,Observer_COPY;
   bool firstrun;
};





#endif
