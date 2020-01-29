#include "LIPMThread.h"
#include <iostream>


LIPMThread::LIPMThread(RobotParameters &rp ) :  OurRobot(rp), DynamicsX(rp), DynamicsY(rp), KalmanX(rp), KalmanY(rp)
{



    KalmanX.uBuffer.push(0.000);
    KalmanY.uBuffer.push(0.000);
    uX=0.000;
    uY=0.000;
    /** Defining the Optimal Gains for the Preview Controller **/
    //Integral Feedback Gain

     //Defining the Optimal Gains for the Preview Controller for Preview Horizon 50 and ComZ=0.268
    //Integral Feedback Gain
    Gi=-61.3668;

    //State Feedback Gain
Gx(0,0) =-2424.5583;
Gx(0,1) =-399.8724;
Gx(0,2) =241.1735;
    //Predicted Reference Gain
 Gd(0) =61.3668;
Gd(1) =99.0257;
Gd(2) =106.6303;
Gd(3) =104.9462;
Gd(4) =100.5372;
Gd(5) =95.4707;
Gd(6) =90.3906;
Gd(7) =85.491;
Gd(8) =80.8238;
Gd(9) =76.3964;
Gd(10) =72.2025;
Gd(11) =68.232;
Gd(12) =64.474;
Gd(13) =60.9175;
Gd(14) =57.5522;
Gd(15) =54.3681;
Gd(16) =51.3557;
Gd(17) =48.5061;
Gd(18) =45.8107;
Gd(19) =43.2614;
Gd(20) =40.8505;
Gd(21) =38.5706;
Gd(22) =36.415;
Gd(23) =34.3769;
Gd(24) =32.4502;
Gd(25) =30.6289;
Gd(26) =28.9075;
Gd(27) =27.2805;
Gd(28) =25.7429;
Gd(29) =24.29;
Gd(30) =22.9172;
Gd(31) =21.6202;
Gd(32) =20.395;
Gd(33) =19.2376;
Gd(34) =18.1443;
Gd(35) =17.1118;
Gd(36) =16.1368;
Gd(37) =15.216;
Gd(38) =14.3466;
Gd(39) =13.5258;
Gd(40) =12.7509;
Gd(41) =12.0194;
Gd(42) =11.329;
Gd(43) =10.6773;
Gd(44) =10.0623;
Gd(45) =9.4819;
Gd(46) =8.9343;
Gd(47) =8.4176;
Gd(48) =7.9302;
Gd(49) =7.4704;
Gd(50) =7.0366;
Gd(51) =6.6275;
Gd(52) =6.2416;
Gd(53) =5.8778;
Gd(54) =5.5347;
Gd(55) =5.2111;
Gd(56) =4.9061;
Gd(57) =4.6186;
Gd(58) =4.3475;
Gd(59) =4.092;
Gd(60) =3.8512;
Gd(61) =3.6243;
Gd(62) =3.4104;
Gd(63) =3.2089;
Gd(64) =3.019;
Gd(65) =2.8402;
Gd(66) =2.6717;
Gd(67) =2.5129;
Gd(68) =2.3634;
Gd(69) =2.2226;
Gd(70) =2.09;
Gd(71) =1.9652;
Gd(72) =1.8476;
Gd(73) =1.7369;
Gd(74) =1.6327;
Gd(75) =1.5346;
Gd(76) =1.4423;
Gd(77) =1.3554;
Gd(78) =1.2736;
Gd(79) =1.1967;
Gd(80) =1.1243;
Gd(81) =1.0561;
Gd(82) =0.99205;
Gd(83) =0.93176;
Gd(84) =0.87505;
Gd(85) =0.82171;
Gd(86) =0.77155;
Gd(87) =0.72438;
Gd(88) =0.68003;
Gd(89) =0.63832;
Gd(90) =0.59912;
Gd(91) =0.56227;
Gd(92) =0.52763;
Gd(93) =0.49507;
Gd(94) =0.46447;
Gd(95) =0.43572;
Gd(96) =0.40871;
Gd(97) =0.38332;
Gd(98) =0.35948;
Gd(99) =0.33708;





    //Initializing Variables
    Integrationfbx=0.000;
    Statefbx=0.000;
    Predictionfbx=0.000;
    Integrationfby=0.000;
    Statefby=0.000;
    Predictionfby=0.000;



    ZMPReferenceX.zero();
    ZMPReferenceY.zero();
    vrpx_d = 0;
    vrpy_d = 0;
    comx_d = 0;
    comy_d = 0;
    comdx_d = 0;
    comdy_d = 0;
    dcmx_d = 0;
    dcmy_d = 0;
    std::cout<<"Online Pattern Planner with Preview Controller Initialized Successfully"<<std::endl;
    firstrun = true;
    Observer_CoMX = 0.5;
	Observer_CoMY = 1.0;
	Observer_COPX = 0.35;
	Observer_COPY = 0.5;
}
void LIPMThread::setInitialState( KVecFloat2 CoM, KVecFloat2 ZMP)
{

    DynamicsX.State(0) = CoM(0);
    DynamicsY.State(0) = CoM(1);
    DynamicsX.State(2) = ZMP(0);
    DynamicsY.State(2) = ZMP(1);

    DynamicsX.zmpstate= ZMP(0);
    DynamicsY.zmpstate= ZMP(1);
    DynamicsX.zmpstate_= ZMP(0);
    DynamicsY.zmpstate_= ZMP(1);

    vrpx_d = DynamicsX.zmpstate;
    vrpy_d = DynamicsY.zmpstate;
    comx_d = DynamicsX.State(0);
    comy_d = DynamicsY.State(0);
    comdx_d = DynamicsX.State(1);
    comdy_d = DynamicsY.State(1);

}








void LIPMThread::Control(boost::circular_buffer<KVecFloat3> & ZmpBuffer, float CoMMeasuredX, float CoMMeasuredY, float CoMMeasuredZ, float ZMPMeasuredX, float ZMPMeasuredY)
{




/* Filter the ZMP Measurement for Delay Noise and Bias */
    KalmanX.Filter(ZMPMeasuredX,CoMMeasuredX);
    KalmanY.Filter(ZMPMeasuredY,CoMMeasuredY);
    //Setting the Reference Signal

    for (unsigned int i = 0; i < PreviewWindow; i++)
    {
        if (i < ZmpBuffer.size())
        {
            ZMPReferenceX(i) = ZmpBuffer[i](0);
            ZMPReferenceY(i) = ZmpBuffer[i](1);
        }
        else
        {
            ZMPReferenceX(i) = ZmpBuffer[ZmpBuffer.size() - 1](0);
            ZMPReferenceY(i) = ZmpBuffer[ZmpBuffer.size() - 1](1);
        }
    }


    //State Feedback Computation
    Statefbx=0.000;
    Statefby=0.000;
    Statefbx=Gx(0,0)*DynamicsX.State(0)+Gx(0,1)*DynamicsX.State(1)+Gx(0,2)*DynamicsX.State(2);
    Statefby=Gx(0,0)*DynamicsY.State(0)+Gx(0,1)*DynamicsY.State(1)+Gx(0,2)*DynamicsY.State(2);


    //Updating the Integral Feedback
    Integrationfbx+=Gi*(DynamicsX.State(2)-ZMPReferenceX(0));
    Integrationfby+=Gi*(DynamicsY.State(2)-ZMPReferenceY(0));


    //Predicted Feedback Computation
    Predictionfbx=0.000;
    Predictionfby=0.000;



    unsigned  int z=0;
    for (unsigned  int i=0; i<PreviewWindow-1; i++)
    {
        Predictionfbx+=Gd(z)*ZMPReferenceX(i+1);
        Predictionfby+=Gd(z)*ZMPReferenceY(i+1);
        z++;
    }




    //Optimal Preview Control
    uX=-Statefbx-Integrationfbx-Predictionfbx;
    uY=-Statefby-Integrationfby-Predictionfby;
    //Updating the Dynamics
   KVecFloat2 errorX=KVecFloat2(CoMMeasuredX,ZMPMeasuredX);
   KVecFloat2 errorY=KVecFloat2(CoMMeasuredY,ZMPMeasuredY);

 //Updating the Dynamics


    DynamicsX.Observer_CoM= Observer_CoMX;
    DynamicsY.Observer_CoM= Observer_CoMY;
    DynamicsX.Observer_COP= Observer_COPX;
    DynamicsY.Observer_COP= Observer_COPY;
    // KVecFloat2 errorX=KVecFloat2(CoMMeasuredX,KalmanX.StatePredict(0));
    // KVecFloat2 errorY=KVecFloat2(CoMMeasuredY,KalmanY.StatePredict(0));



    DynamicsX.Update(uX,errorX);
    DynamicsY.Update(uY,errorY);

    zmpx = DynamicsX.zmpstate_;
    zmpy = DynamicsY.zmpstate_; 
    zmpx_ref = ZMPReferenceX(0);
    zmpy_ref = ZMPReferenceY(0);

    //Estimated COM position
    vrpx_d = DynamicsX.zmpstate;
    vrpy_d = DynamicsY.zmpstate;
    comx_d = DynamicsX.State(0);
    comy_d = DynamicsY.State(0);
    comdx_d = DynamicsX.State(1);
    comdy_d = DynamicsY.State(1);
    vrpdx_d = uX;
    vrpdy_d = uY;

    dcmx_d = comx_d + 1.0/OurRobot.getWalkParameter(omega)*comdx_d;
    dcmy_d = comy_d + 1.0/OurRobot.getWalkParameter(omega)*comdy_d;

    KalmanX.uBuffer.push(DynamicsX.zmpstate-DynamicsX.zmpstate_);
    KalmanY.uBuffer.push(DynamicsY.zmpstate-DynamicsY.zmpstate_);

}



DynamicsLIPM::DynamicsLIPM(RobotParameters &robot): OurRobot(robot)
{
/** Defining the System Dynamics Augmented with the Observer Structure **/

    Ad(0,0)=1.0000;
    Ad(0,1)=OurRobot.getWalkParameter(Ts);
    Ad(0,2)=0.000;
    Ad(1,0)=(OurRobot.getWalkParameter(g)/OurRobot.getWalkParameter(ComZ))*OurRobot.getWalkParameter(Ts);
    Ad(1,1)=1.0000;
    Ad(1,2)=-Ad(1,0);
    Ad(2,0)=0.0000;
    Ad(2,1)=0.0000;
    Ad(2,2)=1.0000;
    Ad(0,3)=0.000;
    Ad(1,3)=0.000;
    Ad(2,3)=0.000;
    Ad(3,0)=0.000;
    Ad(3,1)=0.000;
    Ad(3,2)=0.000;
    Ad(3,3)=1.000;


    Bd(0)=0;
    Bd(1)=0;
    Bd(2)=OurRobot.getWalkParameter(Ts);
    Bd(3)=0;
    Cd(0)=0;
    Cd(1)=0;
    Cd(2)=1;


/** Defining the Optimal Gain for the Observer **/
/** PREVIEW CONTROLLER OBSERVER **/



    L(0,0)=0.1161;
    L(0,1)=0.0301;
    L(1,0)=-0.1401;
    L(1,1)=-0.3260 ;
    L(2,0)=2.3890;
    L(2,1)=-0.5792;
    L(3,0)=-0.0185;
    L(3,1)=0.0241;

    Lcom(0,0)=L(0,0);
    Lcom(1,0)=L(1,0);
    Lcom(2,0)=L(2,0);
    Lcom(3,0)=L(3,0);



error.zero();
y_.zero();
State.zero();
zmpstate=0.000;
zmpstate_=0.000;

temp.zero();


Observer_CoM = 0.0;
Observer_COP = 0.0;
std::cout<<"Dynamics Initialized Successfully"<<std::endl;


}


void DynamicsLIPM::Update(float u,KVecFloat2 y)
{
/** Updating the Dynamics **/
error.zero();
if(fabs(y(1))>1e-10)
{
error = y - KVecFloat2(State(0),State(2)+State(3));
error(0)*=Observer_CoM;
error(1)*=Observer_COP;
y_=y;
//std::cout<<"Observer_CoM: "<<Observer_CoM<<" Observer_COP: "<<Observer_COP<<std::endl;
    State=Ad*State;
    temp=Bd;
    temp.scalar_mult(u);
    State+=temp;
    State+=L*error;
}
else
{
    error(0) = y(0) -  State(0);
    error(0)*=Observer_CoM;
    error(1) = y_(1) - (State(2)+State(3));
    error(1)*=Observer_COP;
    State=Ad*State;
    temp=Bd;
    temp.scalar_mult(u);
    State+=temp;
    State+=L*error;
}
    zmpstate_ = zmpstate;
    zmpstate= State(2);
}



KalmanLIPM::KalmanLIPM(RobotParameters &robot):OurRobot(robot)
{
    /*Akalman(0,0)=1.000;
     Akalman(0,1)=0;//-OurRobot.getWalkParameter(Ts);
     Akalman(1,0)=0.000;
     Akalman(1,1)=1.000;*/
    Bkalman.zero();
    Bkalman(0)=1.000;//OurRobot.getWalkParameter(Ts);
    StateKalman.zero();
    ProcessNoise.zero();
    ProcessNoise(0,0)=4*1e-5; //5*1e-7;
    s.zero();
    P.zero();
    Kgain.zero();
    P(0,0)=1e-10;
    MeasurementNoise.identity();
    
    CoM_Noise = 0.0005;
    COP_Noise = 0.1;
    MeasurementNoise(0,0) = CoM_Noise;
    MeasurementNoise(1,1) = COP_Noise;
    Ckalman.zero();
    Ckalman(0,0)=1.000;
    Ckalman(1,0)=1.000;
    StatePredict.zero();
    ykalman.zero();
    
    std::cout<<"ZMP Delayed Kalman Filter Initialized Successfully"<<std::endl;
    
    
    
}


/** Kalman filter to  deal with Delay, and  Noise **/
void KalmanLIPM::Filter(float ZMPMeasured,float CoMMeasured)
{
    
    MeasurementNoise(0,0) = CoM_Noise;
    MeasurementNoise(1,1) = COP_Noise;
    
    /** Predict **/
    
    StateKalman=StateKalman+Bkalman*uBuffer.front();
    /** Estimated ZMP from the COM measurement
     * using the Cart and Table Model
     **/
    float ct1=0.000,ct2=0.000;
    float zmpfromcom=0.000;
    
    bool doup=false ;
    if(fabs(ZMPMeasured)>1e-15)
    {
        P=P+ProcessNoise;
        doup=true;
        
        if(combuffer.size()==2)
        {
            
            
            ct2=combuffer.front();
            combuffer.pop();
            ct1=combuffer.front();
            combuffer.pop();
            combuffer.push(ct1);
            
            
            float comddot=(CoMMeasured-2*ct1+ct2)/(OurRobot.getWalkParameter(Ts)*OurRobot.getWalkParameter(Ts) );
            zmpfromcom=CoMMeasured-(OurRobot.getWalkParameter(ComZ)/OurRobot.getWalkParameter(g))*comddot;
        }
        
    }
    
    combuffer.push(CoMMeasured);
    
    /** Update **/
    //StateKalman.prettyPrint();
    if(doup)
    {
        /** innovation value **/
        ykalman=KVecFloat2(ZMPMeasured,zmpfromcom);
        ykalman+=(Ckalman*(StateKalman)).scalar_mult(-1.0);
        
        s=Ckalman*P*Ckalman.transp()+MeasurementNoise;
        s.fast_invert();
        Kgain=(P*Ckalman.transp())*s;
        StateKalman+=Kgain*ykalman;
        //StateKalman.prettyPrint();
    }
    
    //StateKalman.prettyPrint();
    
    Kgain.scalar_mult(-1.0);
    P+=Kgain*Ckalman*P;
    if(combuffer.size()>2)
        combuffer.pop();
    
    
    if(uBuffer.size()>ZMPKALMANDELAY)
        uBuffer.pop();
    
    
    /** Getting Rid of the ZMP delay **/
    StatePredict=StateKalman;
    unsigned bufsize=uBuffer.size();
    for(unsigned i=0; i<bufsize; i++)
    {
        StatePredict=StatePredict+Bkalman*uBuffer.front();
        uBuffer.push(uBuffer.front());
        uBuffer.pop();
    }
    
    /** ------------------------------------------------------------- **/
}
