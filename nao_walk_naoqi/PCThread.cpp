#include "PCThread.h"
#include <iostream>


PCThread::PCThread(RobotParameters &rp ) :  OurRobot(rp), DynamicsX(rp), DynamicsY(rp), KalmanX(rp), KalmanY(rp)
{



    KalmanX.uBuffer.push(0.000);
    KalmanY.uBuffer.push(0.000);
    uX=0.000;
    uY=0.000;
    /** Defining the Optimal Gains for the Preview Controller **/
    //Integral Feedback Gain
    /*
   Gi=2.2069e+03;
    //State Feedback Gain
    Gx(0,0)=1.0e+04 * 7.3550 ;
    Gx(0,1)=1.0e+04 * 1.2624;
    Gx(0,2)=1.0e+04 * 0.0147;
    //Predicted Reference Gain
   Gd(0)=-2.2069;
   Gd(1)=-3.4473;
   Gd(2)=-3.9953;
   Gd(3)=-3.9547;
   Gd(4)=-3.7037;
   Gd(5)=-3.4391;
   Gd(6)=-3.2096;
   Gd(7)=-3.0095;
   Gd(8)=-2.8273;
   Gd(9)=-2.6567;
   Gd(10)=-2.4959;
   Gd(11)=-2.3445;
   Gd(12)=-2.2021;
   Gd(13)=-2.0683;
   Gd(14)=-1.9427;
   Gd(15)=-1.8247;
   Gd(16)=-1.7139;
   Gd(17)=-1.6098;
   Gd(18)=-1.5121;
   Gd(19)=-1.4202;
   Gd(20)=-1.3340;
   Gd(21)=-1.2530;
   Gd(22)=-1.1769;
   Gd(23)=-1.1054;
   Gd(24)=-1.0383;
   Gd(25)=-0.9752;
   Gd(26)=-0.9160;
   Gd(27)=-0.8604;
   Gd(28)=-0.8081;
   Gd(29)=-0.7590;
   Gd(30)=-0.7129;
   Gd(31)=-0.6696;
   Gd(32)=-0.6290;
   Gd(33)=-0.5908;
   Gd(34)=-0.5549;
   Gd(35)=-0.5212;
   Gd(36)=-0.4896;
   Gd(37)=-0.4598;
   Gd(38)=-0.4319;
   Gd(39)=-0.4057;
   Gd(40)=-0.3810;
   Gd(41)=-0.3579;
   Gd(42)=-0.3362;
   Gd(43)=-0.3157;
   Gd(44)=-0.2966;
   Gd(45)=-0.2786;
   Gd(46)=-0.2616;
   Gd(47)=-0.2457;
   Gd(48)=-0.2308;
   Gd(49)=-0.2168;
   Gd(50)=-0.2036;
    Gd(51)=-0.1913;
    Gd(52)=-0.1797;
    Gd(53)=-0.1687;
    Gd(54)=-0.1585;
    Gd(55)=-0.1489;
    Gd(56)=-0.1398;
    Gd(57)=-0.1313;
    Gd(58)=-0.1234;
    Gd(59)=-0.1159;
      Gd(60)=-0.1088;
        Gd(61)=-0.1022;
       Gd(62)=-0.0960;
       Gd(63)=-0.0902;
       Gd(64)=-0.0847;
        Gd(65)=-0.0796;
         Gd(66)=-0.0747;
         Gd(67)=-0.0702;
          Gd(68)=-0.0659;
           Gd(69)=-0.0619;
           Gd(70)=-0.0582;
           Gd(71)= -0.0546;
           Gd(72)= -0.0513;
           Gd(73)= -0.0482;
           Gd(74)= -0.0453;
           Gd(75)= -0.0425;
           Gd(76)= -0.0399;
           Gd(77)= -0.0375;
           Gd(78)= -0.0352;
           Gd(79)= -0.0331;
           Gd(80)= -0.0311;
           Gd(81)= -0.0292;
           Gd(82)= -0.0274;
           Gd(83)= -0.0258;
           Gd(84)= -0.0242;
           Gd(85)= -0.0227;
           Gd(86)= -0.0213;
           Gd(87)= -0.0201;
           Gd(88)= -0.0188;
           Gd(89)= -0.0177;
           Gd(90)= -0.0166;
           Gd(91)= -0.0156;
           Gd(92)= -0.0147;
           Gd(93)= -0.0138;
           Gd(94)= -0.0129;
           Gd(95)= -0.0121;
           Gd(96)= -0.0114;
           Gd(97)= -0.0107;
           Gd(98)= -0.0101;
           Gd(99)= -0.0095;
    Gd.scalar_mult(1.0e+03);
    */


    Gi=858.9552;
    //State Feedback Gain
    Gx(0,0)=1.0e+04 * 3.0043 ;
    Gx(0,1)=1.0e+04 * 0.5404;
    Gx(0,2)=1.0e+04 * 0.0090;
    //Predicted Reference Gain
    Gd(0)=-0.8590;
    Gd(1)=-1.0544;
    Gd(2)=-1.2600;
    Gd(3)=-1.3975;
    Gd(4)=-1.4522;
    Gd(5)=-1.4390;
    Gd(6)=-1.3819;
    Gd(7)=-1.3027;
    Gd(8)=-1.2168;
    Gd(9)=-1.1332;
    Gd(10)=-1.0562;
    Gd(11)=-0.9867;
    Gd(12)=-0.9241;
    Gd(13)=-0.8674;
    Gd(14)=-0.8153;
    Gd(15)=-0.7670;
    Gd(16)=-0.7217;
    Gd(17)=-0.6791;
    Gd(18)=-0.6390;
    Gd(19)=-0.6011;
    Gd(20)=-0.5654;
    Gd(21)=-0.5317;
    Gd(22)=-0.5000;
    Gd(23)=-0.4702;
    Gd(24)=-0.4422;
    Gd(25)=-0.4158;
    Gd(26)=-0.3910;
    Gd(27)=-0.3677;
    Gd(28)=-0.3458;
    Gd(29)=-0.3252;
    Gd(30)=-0.3059;
    Gd(31)=-0.2876;
    Gd(32)=-0.2705;
    Gd(33)=-0.2544;
    Gd(34)=-0.2392;
    Gd(35)=-0.2250;
    Gd(36)=-0.2116;
    Gd(37)=-0.1990;
    Gd(38)=-0.1871;
    Gd(39)=-0.1760;
    Gd(40)=-0.1655;
    Gd(41)=-0.1556;
    Gd(42)=-0.1463;
    Gd(43)=-0.1376;
    Gd(44)=-0.1294;
    Gd(45)=-0.1217;
    Gd(46)=-0.1145;
    Gd(47)=-0.1076;
    Gd(48)=-0.1012;
    Gd(49)=-0.0952;
    Gd.scalar_mult(1.0e+03);


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
    Observer_CoMX = 0.7;
	Observer_CoMY = 0.7;
	Observer_COPX = 0.3;
	Observer_COPY = 0.3;
    KalmanX.COP_Noise=0.5;
    KalmanY.COP_Noise=0.5;
    KalmanX.CoM_Noise=0.001;
    KalmanY.CoM_Noise=0.001;
}
void PCThread::setInitialState( KVecFloat2 CoM, KVecFloat2 ZMP)
{

    DynamicsX.State(0) = CoM(0);
    DynamicsY.State(0) = CoM(1);

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








void PCThread::Control(boost::circular_buffer<KVecFloat3> & ZmpBuffer, float CoMMeasuredX, float CoMMeasuredY, float CoMMeasuredZ, float ZMPMeasuredX, float ZMPMeasuredY)
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
    Integrationfbx+=Gi*((DynamicsX.zmpstate)-ZMPReferenceX(0));
    Integrationfby+=Gi*((DynamicsY.zmpstate)-ZMPReferenceY(0));


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


    DynamicsX.Observer_CoM= Observer_CoMX;
    DynamicsY.Observer_CoM= Observer_CoMY;
    DynamicsX.Observer_COP= Observer_COPX;
    DynamicsY.Observer_COP= Observer_COPY;
    KVecFloat2 errorX=KVecFloat2(CoMMeasuredX,KalmanX.StatePredict(0));
    KVecFloat2 errorY=KVecFloat2(CoMMeasuredY,KalmanY.StatePredict(0));
    //KVecFloat2 errorX=KVecFloat2(CoMMeasuredX,ZMPMeasuredX);
    //KVecFloat2 errorY=KVecFloat2(CoMMeasuredY,ZMPMeasuredY);


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
    vrpdx_d = comdx_d - uX/(OurRobot.getWalkParameter(omega)*OurRobot.getWalkParameter(omega));
    vrpdy_d = comdy_d - uY/(OurRobot.getWalkParameter(omega)*OurRobot.getWalkParameter(omega));

    dcmx_d = comx_d + 1.0/OurRobot.getWalkParameter(omega)*comdx_d;
    dcmy_d = comy_d + 1.0/OurRobot.getWalkParameter(omega)*comdy_d;

    KalmanX.uBuffer.push(DynamicsX.zmpstate-DynamicsX.zmpstate_);
    KalmanY.uBuffer.push(DynamicsY.zmpstate-DynamicsY.zmpstate_);

}



Dynamics::Dynamics(RobotParameters &robot): OurRobot(robot)
{
/** Defining the System Dynamics Augmented with the Observer Structure **/

Ad(0,0)=1.0000;
Ad(0,1)=OurRobot.getWalkParameter(Ts);
Ad(0,2)=OurRobot.getWalkParameter(Ts)*OurRobot.getWalkParameter(Ts)/2.0000;
Ad(1,0)=0.0000;
Ad(1,1)=1.0000;
Ad(1,2)=Ad(0,1);
Ad(2,0)=0.0000;
Ad(2,1)=0.0000;
Ad(2,2)=1.0000;
/*----------------------*/
Ad(0,3)=0.0000;
Ad(1,3)=0.0000;
Ad(2,3)=0.0000;
Ad(3,0)=0.0000;
Ad(3,1)=0.0000;
Ad(3,2)=0.0000;
Ad(3,3)=1.0000;


Bd(0)=OurRobot.getWalkParameter(Ts)*OurRobot.getWalkParameter(Ts)*OurRobot.getWalkParameter(Ts)/6.0000;
Bd(1)=OurRobot.getWalkParameter(Ts)*OurRobot.getWalkParameter(Ts)/2.0000;
Bd(2)=OurRobot.getWalkParameter(Ts);
Bd(3)=0.0000;

Cd(0)=1.0000;
Cd(1)=0.0000;
Cd(2)=-(OurRobot.getWalkParameter(ComZ))/OurRobot.getWalkParameter(g);

/** Defining the Optimal Gain for the Observer **/
/** PREVIEW CONTROLLER OBSERVER **/



  L(0,0)=0.1128;
    L(0,1)=0.0606;
    L(1,0)=-0.5004;
    L(1,1)=-0.2043;
    L(2,0)=-12.8674;
    L(2,1)=-22.9822;
    L(3,0)=-0.1403;
    L(3,1)=0.0821;


State.zero();
zmpstate=0.000;
zmpstate_=0.000;

temp.zero();


Observer_CoM = 0.0;
Observer_COP = 0.0;
std::cout<<"Dynamics Initialized Successfully"<<std::endl;


}


void Dynamics::Update(float u,KVecFloat2 error)
{
/** Updating the Dynamics **/
error-=KVecFloat2(State(0),(Cd(0)*State(0)+Cd(2)*State(2)+State(3)));
error(0)*=Observer_CoM;
error(1)*=Observer_COP;
//std::cout<<"Observer_CoM: "<<Observer_CoM<<" Observer_COP: "<<Observer_COP<<std::endl;

State=Ad*State;
temp=Bd;
temp.scalar_mult(u);
State+=temp;
State+=L*error;
zmpstate_ = zmpstate;
zmpstate=(Cd(0)*State(0)+Cd(2)*State(2));
}



Kalman::Kalman(RobotParameters &robot):OurRobot(robot)
{
    /*Akalman(0,0)=1.000;
     Akalman(0,1)=0;//-OurRobot.getWalkParameter(Ts);
     Akalman(1,0)=0.000;
     Akalman(1,1)=1.000;*/
    Bkalman.zero();
    Bkalman(0)=1.000;//OurRobot.getWalkParameter(Ts);
    StateKalman.zero();
    ProcessNoise.zero();
    ProcessNoise(0,0)=5*1e-7; //5*1e-7;
    s.zero();
    P.zero();
    Kgain.zero();
    P(0,0)=1e-10;
    MeasurementNoise.identity();
    
    CoM_Noise = 0.005;
    COP_Noise = 0.05;
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
void Kalman::Filter(float ZMPMeasured,float CoMMeasured)
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
