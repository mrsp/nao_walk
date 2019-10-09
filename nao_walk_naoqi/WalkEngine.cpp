#include "WalkEngine.h"
#define MAX_ID 1000
#define DEF_STEP_ID -1

WalkEngine::WalkEngine(RobotParameters &rp) : NaoLIPM(rp),NaoRobot(rp),stepAnkleQ(rp.getWalkParameter(StepPlanSize)), velocityQ(rp.getWalkParameter(StepPlanSize)),tp(rp), sp(rp), 
NaoPosture(rp), NaoMPCDCM(rp), NaoVRPToCoM(rp), NaoFeetEngine(rp),  zmpDist(rp), CoM_ac(rp),  copKF(rp) // flog("log",RAW,20)
{
    
    FSRDataInit = false;
    JointDataInit = false;
    IMUDataInit = false;
    nipmEKF = new CoMEKF();
    nipmEKF->com_q = NaoRobot.getWalkParameter(com_q);
    nipmEKF->comd_q = NaoRobot.getWalkParameter(comd_q);
    nipmEKF->com_r = NaoRobot.getWalkParameter(com_r);
    nipmEKF->comdd_r = NaoRobot.getWalkParameter(comdd_r);
    nipmEKF->fd_q = NaoRobot.getWalkParameter(fd_q);
    nipmEKF->init();
    nipmEKF->setdt(NaoRobot.getWalkParameter(Ts));
    nipmEKF->setParams(NaoRobot.getWalkParameter(mass), NaoRobot.getWalkParameter(I_xx),
                      NaoRobot.getWalkParameter(I_yy), NaoRobot.getWalkParameter(g));
  
    Inertia.setZero();  
    Inertia(0,0) = NaoRobot.getWalkParameter(I_xx);
    Inertia(1,1) = NaoRobot.getWalkParameter(I_yy);
    Inertia(2,2) = 0.0;
    
 
    noContactLeft = false;
    noContactRight = false;
    
    /** Walk Engine Execution **/
    
    stepPhaseId = 0;
    ci.targetSupport=KDeviceLists::SUPPORT_LEG_NONE;
    ci.step_id=DEF_STEP_ID;

    executedStep = ci;
    _isFootStepExecuted = false;
    isFSRInitialized = false;
    isEngineInitialized = false;

    /** WalkEngine Step **/
    currentstep=0;
    ci_id = 0;
    startup = 0;
    /** Randomly picking the support leg **/
    supportleg = KDeviceLists::SUPPORT_LEG_LEFT;
    oldsupportleg = supportleg;
    chainsupport=KDeviceLists::CHAIN_L_LEG;
    chainswing =  KDeviceLists::CHAIN_R_LEG;
    
    

    Tib_eig.setIdentity();
    Tis_eig.setIdentity();
    Tssprime_eig.setIdentity();
    Tbs_eig.setIdentity();

    /** Kinematics with KMat **/
    /** Initializing the Transformation matrices **/
    Tir.identity();
    Til.identity();
    Tis.identity();
    Tsp.identity();
    Tssprime.identity();
    Tip.identity();
    Tps.identity();
    AccW.setZero();
    
    
    /** Transformation from Pevlis to Support Leg  **/
    Tsp=nkin.getForwardEffector((NAOKinematics::Effectors)KDeviceLists::CHAIN_L_LEG);
    Tps=Tsp;
    /** Transformation from Support Leg
     NaoRobot.getWalkParameter(I_yy), NaoRobot.getWalkParameter(g)); to Inertial Frame of Reference **/
    /** Kinematics Instances **/
    NAOKinematics::FKvars v;
    
    v.p=Tsp.getTranslation();
    v.a=Tsp.getEulerAngles();
    v.p(0)=0;
    v.p(2)=0;
    
    v.a(0)=0;
    v.a(1)=0;
    v.a(2)=0;
    Tis=NAOKinematics::getTransformation(v);
    
    /** Transformation from Support Leg to Pelvis **/
    Tsp.fast_invert();


    if(tp.firstrun)
    {
                tp.firstrun = false;
                /** Compute Reference Points in the x-y plane with orientation about the vertical axis z **/
                planL=getPositionInertial((NAOKinematics::Effectors)KDeviceLists::CHAIN_L_LEG);
                planR=getPositionInertial((NAOKinematics::Effectors)KDeviceLists::CHAIN_R_LEG);
                //planL = KVecFloat3(NaoRobot.getWalkParameter(HX),NaoRobot.getWalkParameter(H0),0.0);
                //planR = KVecFloat3(NaoRobot.getWalkParameter(HX),-NaoRobot.getWalkParameter(H0),0.0);

                /** Feet Engine **/
                NaoFeetEngine.setFootStartXYTheta(planL, LeftFoot);
                NaoFeetEngine.setFootDestXYTheta(planL, LeftFoot);
                NaoFeetEngine.setFootStartZ(0.00,LeftFoot);
                NaoFeetEngine.setFootDestZ(0.00,LeftFoot);
                NaoFeetEngine.setFootXYTheta(planL,LeftFoot);
                
                
                NaoFeetEngine.setFootStartXYTheta(planR, RightFoot);
                NaoFeetEngine.setFootDestXYTheta(planR, RightFoot);
                NaoFeetEngine.setFootStartZ(0.00,RightFoot);
                NaoFeetEngine.setFootDestZ(0.00,RightFoot);
                NaoFeetEngine.setFootXYTheta(planR,RightFoot);
                /** Trajectory Planner **/
                tp.init(planL,planR);
    }
    
    CoMi.zero();
    CoMs.zero();
    CoM_c.setZero();

    
    
    postureInitialized = false;

    /** GAIT INDICATORS **/
    rightEarlyContact=false;
    leftEarlyContact=false;
    rightLateContact=false;
    leftLateContact=false;
    GroundContact=false;
    inAir=false;
    
    /** VERTICAL GROUND REACTION FORCES **/
    forceL.setZero();
    forceR.setZero();
    copsrel.setZero();
    fis.setZero();
    CoMm.setZero();
    copi.setZero();
    copi_cropped.setZero();
    copl_cropped.setZero();
    copr_cropped.setZero();
    Gyrodot.setZero();
    Gyro_.setZero();
    GyroW.setZero();
    firstGyrodot = true;
    leftGRF_LPF =  new butterworthLPF();
    rightGRF_LPF = new butterworthLPF();
    
    coplx_LPF = new butterworthLPF();
    coply_LPF = new butterworthLPF();
    coprx_LPF = new butterworthLPF();
    copry_LPF = new butterworthLPF();
    
    AccX_LPF =  new butterworthLPF();
    AccY_LPF = new butterworthLPF();
    AccZ_LPF = new butterworthLPF();
    GyroX_LPF = new butterworthLPF();
    GyroY_LPF = new butterworthLPF();
    

    AccX_HPF =  new butterworthHPF();
    AccY_HPF = new butterworthHPF();
    AccZ_HPF = new butterworthHPF();
    GyroX_HPF = new butterworthHPF();
    GyroY_HPF = new butterworthHPF();


    leftGRF_LPF->init("LeftFoot",1.0/NaoRobot.getWalkParameter(Ts),2.5);
    rightGRF_LPF->init("RightFoot",1.0/NaoRobot.getWalkParameter(Ts),2.5);
    
    coplx_LPF->init("LeftCOPX", 1.0/NaoRobot.getWalkParameter(Ts),1.5);
    coply_LPF->init("LeftCOPY", 1.0/NaoRobot.getWalkParameter(Ts),1.5);
    coprx_LPF->init("RightCOPX", 1.0/NaoRobot.getWalkParameter(Ts),1.5);
    copry_LPF->init("RightCOPY", 1.0/NaoRobot.getWalkParameter(Ts),1.5);
    
    AccX_LPF->init("AccelerometerX", 1.0/NaoRobot.getWalkParameter(Ts),4.0);
    AccY_LPF->init("AccelerometerY", 1.0/NaoRobot.getWalkParameter(Ts),4.0);
    AccZ_LPF->init("AccelerometerZ", 1.0/NaoRobot.getWalkParameter(Ts),4.0);
    GyroX_LPF->init("GyroX", 1.0/NaoRobot.getWalkParameter(Ts),4.0);
    GyroY_LPF->init("GyroY", 1.0/NaoRobot.getWalkParameter(Ts),4.0);
    
    AccX_HPF->init("AccelerometerX", 1.0/NaoRobot.getWalkParameter(Ts),0.4);
    AccY_HPF->init("AccelerometerY", 1.0/NaoRobot.getWalkParameter(Ts),0.4);
    AccZ_HPF->init("AccelerometerZ", 1.0/NaoRobot.getWalkParameter(Ts),0.4);
    GyroX_HPF->init("GyroX", 1.0/NaoRobot.getWalkParameter(Ts),0.4);
    GyroY_HPF->init("GyroY", 1.0/NaoRobot.getWalkParameter(Ts),0.4);
    
    alljoints.resize(KDeviceLists::NUMOFJOINTS, 0);
   /* 
    for (int j = 0; j < KDeviceLists::NUMOFJOINTS; j++)
    {
        JointKF[j].initialize(NaoRobot);
        
    }
    */
    
    
    std::cout<<"WalkEngine Initialized Successfully"<<std::endl;
    
    isEstimationLoopInitialized = false;
    transitionSI_idx = 0;
    step_id = 1;
    last_step_id=0;
}



void WalkEngine::resetBuffers()
{
    while(stepAnkleQ.size()>0)
        stepAnkleQ.pop_front();
}

void WalkEngine::addWalkInstruction(WalkInstruction &i)
{
    i.step_id = assignStepID();
    stepAnkleQ.push_back(i);
}

void WalkEngine::addWalkInstruction()
{
    /** Initial Walking Instruction **/
    WalkInstruction i;

    /** Randomly chosen Left Leg as Initial Support Leg **/
    i.targetSupport=supportleg;
    
    i.target = (supportleg == KDeviceLists::SUPPORT_LEG_RIGHT)?planL:planR;

    /** ZMP in the Middle of Convex Hull **/
    i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
    /** Number of Discrete Time steps of the Initial Walking Instruction **/
    i.steps=NaoRobot.getWalkParameter(Init_instructions);    
    i.step_id=DEF_STEP_ID;
    /** Adding the Walking Instruction to the Walking Buffer for Execution **/
    stepAnkleQ.push_back(i);
}

int WalkEngine::assignStepID()
{
    step_id++;
    return step_id;
}

void WalkEngine::feed()
{
    if((ci.targetSupport == KDeviceLists::SUPPORT_LEG_NONE || currentstep == ci.steps))
    {

        if(ci_id > 8 && !postureInitialized)
        {        
            postureInitialized = true;
            cout<<"Posture Initialized Successfully"<<endl;
        }
        else if(!postureInitialized)
        {
            ci_id++;
        }
        
        if(NaoRobot.getWalkParameter(velocityControl) && velocityQ.size()>0 && postureInitialized && !_isFootStepExecuted)
        {
            unsigned int jjj = 0;
            WalkInstruction pi;

            //while(stepAnkleQ.size()>0)
            //       stepAnkleQ.pop_front();

            while(velocityQ.size()>0)
            {
                if(jjj==0)
                {
                    if(stepAnkleQ.size()>0)
                        pi = sp.planStep2D(velocityQ.front(), stepAnkleQ.back());
                    else
                        pi = sp.planStep2D(velocityQ.front(), getCurrStep());

                }
                else
                {   
                    pi = sp.planStep2D(velocityQ.front(),pi);

                }
                addWalkInstruction(pi);
                jjj++;
                velocityQ.pop_front();
            }


          
        }
        if(NaoRobot.getWalkParameter(velocityControl))
        {
            while(stepAnkleQ.size()<3)
            {
                WalkInstruction temp_i = sp.planStep2D(KVecFloat3(0,0,0),stepAnkleQ.back());
                addWalkInstruction(temp_i);
            }
        }
        //No step available --> Balance Mode
        if(stepAnkleQ.size() == 0)
            addWalkInstruction();



        executedStep = ci;
        if(!_isFootStepExecuted)
        {
            //ASK REPLAN FUNCTION of WalkEngine
            unsigned int j=0;
            if(tp.planAvailable)
                tp.emptyPlan();
            

            while(tp.stepAnkleQ.size()<3)
            {
                if(j<stepAnkleQ.size())
                    tp.stepAnkleQ.push(stepAnkleQ[j]);
                else
                {
                    /** Initial Walking Instruction **/
                    WalkInstruction i = stepAnkleQ.back();
                    /** ZMP in the Middle of Convex Hull **/
                    i.targetZMP=KDeviceLists::SUPPORT_LEG_BOTH;
                    /** Number of Discrete Time steps of the Initial Walking Instruction **/
                    i.steps=NaoRobot.getWalkParameter(Init_instructions);
//                  i.step_id++;
                    i.step_id=DEF_STEP_ID;
                    /** Adding the Walking Instruction to the Walking Buffer for Execution **/
                    tp.stepAnkleQ.push(i);
                }
                j++;
            }
            tp.plan(KVecFloat2(nipmEKF->DCM_lp(0),nipmEKF->DCM_lp(1)),   KVecFloat2(copi(0),copi(1)), nipmEKF->comZ, Angle(0),Angle(1),NaoRobot.getWalkParameter(StepPlanAdjustment),postureInitialized);

            // if(postureInitialized)
            //     tp.plan(KVecFloat2(nipmEKF->DCM_lp(0),nipmEKF->DCM_lp(1)), KVecFloat2(NaoLIPM.vrpx_d,NaoLIPM.vrpy_d),NaoRobot.getWalkParameter(StepPlanAdjustment));
            // else
            //     tp.plan(KVecFloat2(nipmEKF->DCM_lp(0),nipmEKF->DCM_lp(1)),  KVecFloat2(copi(0),copi(1)),false);

            _isFootStepExecuted=true;
            stepAnkleQ.pop_front();
        }

        if(stepPhaseId>0){
            stepPhaseId = 0;
            _isFootStepExecuted=false;
        }
        else
            stepPhaseId++;

        ci = tp.walkInstructionbuffer.front();
        tp.walkInstructionbuffer.pop();

        //ci = tp.getCurrWalkInstruction();
        rightEarlyContact=false;
        leftEarlyContact=false;
        
        /** Check if double support phase **/
        if(ci.phase == double_support)
            double_support_id=true;
        else
            double_support_id=false;
        
        if(ci.targetSupport == KDeviceLists::SUPPORT_LEG_LEFT)
            planR = ci.target;
        else if(ci.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            planL = ci.target;
        

        setCurrStep(ci);        
        currentstep=0;
        
    }
}

void WalkEngine::setCurrStep(const WalkInstruction &i)
{
    si.targetSupport = ci.targetSupport;
    chainsupport = (ci.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)?KDeviceLists::CHAIN_R_LEG:KDeviceLists::CHAIN_L_LEG;
    si.targetZMP = ci.targetZMP;
    si.pose = getPositionInertial((NAOKinematics::Effectors)chainsupport);
    si.step_id = i.step_id;
    
    if(cstep_id!=DEF_STEP_ID)
    {
        last_step_id = cstep_id;
    }
    cstep_id= i.step_id;
}

WalkInstruction WalkEngine::getCurrStep()
{

    return ci;
}

void WalkEngine::computeGyrodot() {
	if (!firstGyrodot) {
		//Compute numerical derivative
		Gyrodot = (Gyro - Gyro_)/NaoRobot.getWalkParameter(Ts);
	} 
	else {
		Gyrodot.setZero();
		firstGyrodot = false;
	}
	Gyro_ = Gyro;
}


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* ---------------------------------------- RUNNING IN REAL TIME -----------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/**
 * This function is the main function computing the Walk Trajectories
 **/
std::vector<float> WalkEngine::runStep()
{ 
    if(isEstimationLoopInitialized)
    {
        /** CHECK FOR GROUND CONTACT **/
        if(forceL(2)+forceR(2)>NaoRobot.getWalkParameter(Ground_Contact_threshold))
        {
            GroundContact=true;
            inAir=false;
        }
        else
            GroundContact=false;
        
        
        
        /** NO GROUND CONTACT **/
        if(!GroundContact && currentstep==ci.steps && startup!=0)
            inAir=true;        /** Transform CoM in Inertial Frame of Reference **/






        /** PrepareBuffers, Transformations, and targets **/
        feed();
        oldsupportleg=supportleg;
        //determineLegContact();
        /** Get the Transformation from pelvis to support **/
        supportleg = ci.targetSupport;

        updateTFs();

        stateEstimationLoop();

        /** Beginning of command to be executed **/
        if(currentstep==0)
        {
            
            NaoFeetEngine.setFootStartXYTheta(NaoFeetEngine.FootL,LeftFoot);
            NaoFeetEngine.setFootStartXYTheta(NaoFeetEngine.FootR,RightFoot);
            NaoFeetEngine.setFootStartZ(NaoFeetEngine.FootRz,RightFoot);
            NaoFeetEngine.setFootStartZ(NaoFeetEngine.FootLz,LeftFoot);
           
            /*
            NaoFeetEngine.setFootStartXYTheta(getPositionInertial((NAOKinematics::Effectors)KDeviceLists::CHAIN_L_LEG),LeftFoot);
            NaoFeetEngine.setFootStartXYTheta(getPositionInertial((NAOKinematics::Effectors)KDeviceLists::CHAIN_R_LEG),RightFoot);
            NaoFeetEngine.setFootStartZ(0.0,RightFoot);
            NaoFeetEngine.setFootStartZ(0.0,LeftFoot);
            */
        }

        Calculate_Desired_COM();
        leg_joint_targets.clear();
        leg_joint_targets = Calculate_IK();
    }
    else
    {
          updateTFs();
          stateEstimationLoop();
          leg_joint_targets.clear();
          leg_joint_targets = Calculate_IK0();
    }

    if(!isEngineInitialized)
        isEngineInitialized = true;

    return  leg_joint_targets;
    
}

void WalkEngine::updateTFs()
{
    right_support_id = supportleg == KDeviceLists::SUPPORT_LEG_RIGHT;
    chainsupport = (right_support_id == true)?KDeviceLists::CHAIN_R_LEG:KDeviceLists::CHAIN_L_LEG;
    /**  Transformation from Support to Pelvis **/
    Tsp = nkin.getForwardEffector((NAOKinematics::Effectors) chainsupport);  //From Torso to Support Foot -- Tps
    Tps = Tsp;
    Tsp.fast_invert();  //From Support Foot  to Torso

    /** Get the Transformation from the support leg to the swing leg **/    
    chainswing = (right_support_id == true)?KDeviceLists::CHAIN_L_LEG:KDeviceLists::CHAIN_R_LEG;
    //Needed in COP transformation to Inertial
    Tssprime = nkin.getForwardFromTo(
                                     (NAOKinematics::Effectors)chainsupport,
                                     (NAOKinematics::Effectors)chainswing
                                     );
    /** Compute the new Tis after leg exchange **/
    if(oldsupportleg != supportleg)
    {
        //NAOKinematics::FKvars t;
        //NAOKinematics::kmatTable a = Tssprime ;
        
        //a.fast_invert();
        
        //std::cout<<"Leg Switching Happening"<<std::endl;
        //Tis *= a;
        
        
        if(right_support_id == false)                           //** TRY THIS OUT  -----------------------   USING IDEAL TIS
            Tis = Til;
        else
            Tis = Tir;
        
    }

    
    /** Get current Com as Measured by Kinematics, in Inertial Frame, in meters and milimeters **/
    
    /**  Transformation from Inertial Frame of Reference  to Pelvis frame **/
    Tip=Tis*Tsp;
    NAOKinematics::FKvars v;
    v.p=Tip.getTranslation();
    v.a=Tip.getEulerAngles();
    v.a(0)=Angle(0);
    v.a(1)=Angle(1);
    Tip=NAOKinematics::getTransformation(v);
    Tis=Tip*Tps;

    //if (!isNear(Tip_,Tip,0.002f))
    //    Tip=Tip_;
    
    

}


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*  										Pattern Generator 																									                                    */
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void WalkEngine::stateEstimationLoop()
{
    
    if(IMUDataInit && FSRDataInit && JointDataInit)
    {
        /** Transform CoM in Inertial Frame of Reference **/
        /** measured CoM in milimeters **/
        CoMi = Tip.transform(nkin.calculateCenterOfMass());
        /** measured CoM in meters **/
        CoMm = Vector3f(CoMi(0)*0.001,CoMi(1)*0.001,CoMi(2)*0.001);
        
        Tib_eig = transformKMatToEigen(Tip);
        Tbs_eig = transformKMatToEigen(Tps);
        Tis_eig = transformKMatToEigen(Tis);
        Tssprime_eig = transformKMatToEigen(Tssprime);
        odomTrans = Tib_eig.translation();
        odomQuat =  Quaternionf(Tib_eig.linear());


        
        
        /** Get Center of Pressure, as measured by the FSRs, in Inertial Frame, in meters **/
        copi=getCoP();
        /** IMU FILTERING **/
        /* Low Pass Filter for Accelerometer */
        Acc = Vector3f(AccX_HPF->filter(Acc(0)),AccY_HPF->filter(Acc(1)),AccZ_HPF->filter(Acc(2)));
        Acc = Vector3f(AccX_LPF->filter(Acc(0)),AccY_LPF->filter(Acc(1)),AccZ_LPF->filter(Acc(2)));
        /* Low Pass Filter for Gyro */
        Gyro  = Vector3f(GyroX_HPF->filter(Gyro(0)),GyroY_HPF->filter(Gyro(1)),Gyro(2));
        Gyro  = Vector3f(GyroX_LPF->filter(Gyro(0)),GyroY_LPF->filter(Gyro(1)),Gyro(2));


        if (copKF.firstrun)
        {
            copKF.setInitialState(Vector4d(copi(0),0,copi(1),0));
        }

        if(isEstimationLoopInitialized)
            copKF.predict(Vector2d(tp.ZMPbuffer[0](0),tp.ZMPbuffer[0](1)));
        else
            copKF.predict(Vector2d(0.0,0.0));


        AccW = Tib_eig.linear()*Acc;
        GyroW = Tib_eig.linear()*Gyro;
        computeGyrodot();
        copKF.updateWithCoM(Vector3d(CoMm(0),CoMm(1),CoMm(2)),Vector3d(AccW(0),AccW(1),AccW(2)), Vector3d(GyroW(0),GyroW(1),GyroW(2)));
        copKF.updateWithCOP(Vector2d(copi(0),copi(1)));
    

        if (nipmEKF->firstrun){
            nipmEKF->setCoMPos(CoMm);
            nipmEKF->setCoMExternalForce(Vector3f(0,0,0));
            nipmEKF->firstrun = false;
        }

        nipmEKF->predict(copi,fis, Tib_eig.linear()*Inertia*Gyrodot);
        
        nipmEKF->update(AccW,  CoMm,  Tib_eig.linear()*Gyro, Tib_eig.linear()*Gyrodot);
        //cout<<"CoM E/M"<<endl;
        //cout<<nipmEKF->comX<<" "<<nipmEKF->comY<<" "<<nipmEKF->comZ<<endl;
        //cout<<CoMm<<endl;
        //cout<<"ZMP E/M"<<endl;
        //cout<<copKF.zmpx<<" "<<copKF.zmpy<<endl;
        //cout<<copi(0)<<" "<<copi(1)<<endl;
    }
}

void WalkEngine::Calculate_Desired_COM()
{

  
     /** Run the Pattern Generator and Get Target Com in Inertial Frame**/
    

    /** Angular Momentum Control in Pattern **/
    
    KVecFloat2 CoMp;
    if (GroundContact)
        CoMp=NaoPosture.soleAnguralMomentumStabilizer(Angle(0),CoMm(0),Angle(1),CoMm(1));
    else
        CoMp=KVecFloat2(CoMm(0),CoMm(1));
    


    
  /*
        if(NaoMPCDCM.firstrun)
        {	
            NaoMPCDCM.setInitialState(Vector2f(nipmEKF->DCM(0),nipmEKF->DCM(1)),Vector2f(CoMp(0),CoMp(1)), Vector2f(copKF.zmpx,copKF.zmpy));
            NaoMPCDCM.firstrun = false;


         }
         NaoMPCDCM.Control(tp.ZMPbuffer,Vector2f(nipmEKF->DCM(0),nipmEKF->DCM(1)),Vector2f(CoMp(0),CoMp(1)), Vector2f(copi(0),copi(1)));
         //CoM_c=NaoVRPToCoM.Control(Vector2d(NaoMPCDCM.comx_d,NaoMPCDCM.comy_d),Vector2d(NaoMPCDCM.comdx_d,NaoMPCDCM.comdy_d),Vector2d(NaoMPCDCM.vrpx_d,NaoMPCDCM.vrpy_d),Vector2d(copKF.zmpx,
        //copKF.zmpy), Vector2d(CoMm(0),CoMm(1)));
    
        //CoM_c= NaoVRPToCoM.Control(Vector2d(NaoMPCDCM.comx_d,NaoMPCDCM.comy_d),Vector2d(NaoMPCDCM.vrpx_d,NaoMPCDCM.vrpy_d), Vector2d(copi(0),copi(1)), Angle(0), Angle(1), GyroW(0), GyroW(1), GroundContact);
        CoM_c= NaoVRPToCoM.ControlNoDelay(Vector2d(NaoMPCDCM.comx_d,NaoMPCDCM.comy_d),Vector2d(NaoMPCDCM.vrpx_d,NaoMPCDCM.vrpy_d), Vector2d(copKF.zmpx,copKF.zmpy), Angle(0), Angle(1), GyroW(0), GyroW(1), GroundContact);

    */
	
    
   
        if(NaoLIPM.firstrun)
        {	
            NaoLIPM.setInitialState(KVecFloat2(CoMp(0),CoMp(1)), KVecFloat2(copi(0),copi(1)));
            NaoLIPM.firstrun = false;


         }
    
    //NaoLIPM.Control(tp.ZMPbuffer,nipmEKF->comX,nipmEKF->comY, nipmEKF->comZ,copi(0),copi(1));

    NaoLIPM.Control(tp.ZMPbuffer,CoMp(0),CoMp(1), CoMm(2),copi(0),copi(1));
    // CoM_c= NaoVRPToCoM.Control(Vector2d(NaoLIPM.comx_d,NaoLIPM.comy_d),Vector2d(NaoLIPM.comdx_d,NaoLIPM.comdy_d),Vector2d(NaoLIPM.zmpx_ref,NaoLIPM.zmpy_ref), Vector2d(NaoLIPM.vrpx_d,NaoLIPM.vrpy_d), Vector2d(CoMp(0),CoMp(1)));
    //CoM_c= NaoVRPToCoM.Control(Vector2d(NaoLIPM.comx_d,NaoLIPM.comy_d), Angle(0), Angle(1), GyroW(0), GyroW(1));
    CoM_c = NaoVRPToCoM.ControlNoDelay(Vector2d(NaoLIPM.comx_d,NaoLIPM.comy_d),Vector2d(NaoLIPM.vrpx_d,NaoLIPM.vrpy_d), Vector2d(copi(0),copi(1)), Angle(0), Angle(1), GyroW(0), GyroW(1), GroundContact);

    //NaoLIPM.Control(tp.ZMPbuffer,CoMp(0),CoMp(1), CoMm(2),copKF.zmpx,copKF.zmpy);
    /*
        flog.insert("stepL_ax",tp.stepl_a(0));
        flog.insert("stepL_ay",tp.stepl_a(1));
        flog.insert("stepL_aw",tp.stepl_a(2));
        flog.insert("stepR_ax",tp.stepr_a(0));
        flog.insert("stepR_ay",tp.stepr_a(1));
        flog.insert("stepR_aw",tp.stepr_a(2));

        flog.insert("stepL_bx",tp.stepl_b(0));
        flog.insert("stepL_by",tp.stepl_b(1));
        flog.insert("stepL_bw",tp.stepl_b(2));
        flog.insert("stepR_bx",tp.stepr_b(0));
        flog.insert("stepR_by",tp.stepr_b(1));
        flog.insert("stepR_bw",tp.stepr_b(2));


        flog.insert("PatternZMPx",NaoLIPM.vrpx_d);
        flog.insert("PatternZMPy",NaoLIPM.vrpy_d);
        flog.insert("PatternCoMx",NaoLIPM.comx_d);
        flog.insert("PatternCoMy",NaoLIPM.comy_d);
        flog.insert("PatternDCMx",NaoLIPM.dcmx_d);
        flog.insert("PatternDCMy",NaoLIPM.dcmy_d);
        flog.insert("PatternCoMdx",NaoLIPM.comdx_d);
        flog.insert("PatternCoMdy",NaoLIPM.comdy_d);

        flog.insert("SEROWCoMx",nipmEKF->comX);
        flog.insert("SEROWCoMy",nipmEKF->comY);
        flog.insert("SEROWCoMz",nipmEKF->comZ);
        flog.insert("SEROWDCMx",nipmEKF->DCM(0));
        flog.insert("SEROWDCMy",nipmEKF->DCM(1));
        flog.insert("SEROWCoMdx",nipmEKF->velX);
        flog.insert("SEROWCoMdy",nipmEKF->velY);
        flog.insert("SEROWCoMdz",nipmEKF->velZ);


        flog.insert("COPx",copi(0));
        flog.insert("COPy",copi(1));
        flog.insert("fis",fis(2));
        flog.insert("refZMPx",tp.ZMPbuffer[0](0));
        flog.insert("refZMPy",tp.ZMPbuffer[0](1));
        flog.insert("COMKinx",CoMm(0));
        flog.insert("COMKiny",CoMm(1));
        flog.insert("COMKinz",CoMm(2)); 
        flog.insert("odomx",odomTrans(0));
        flog.insert("odomy",odomTrans(1));
        flog.insert("odomz",odomTrans(2));
        flog.insert("odomqx",odomQuat.x());
        flog.insert("odomqy",odomQuat.y());
        flog.insert("odomqz",odomQuat.z());
        flog.insert("odomqw",odomQuat.w());
        flog.insert("Accx",AccW(0));
        flog.insert("Accy",AccW(1));
        flog.insert("Accz",AccW(2));
        flog.insert("Gyrox",GyroW(0));
        flog.insert("Gyroy",GyroW(1));
        flog.insert("Gyroz",GyroW(2));
        flog.periodic_save();
    */
   
    
     
  

    /** Pop the used ZMP **/
    tp.ZMPbuffer.pop_front();
}


/** --------------------------------------------------------------- **/
/** --------------------------------------------------------------- **/
/** --------------------------------------------------------------- **/
/** --------------------------------------------------------------- **/






/**
 * This function computes the joint angles that the actuators must be set to
 **/

std::vector<float> WalkEngine::Calculate_IK0()
{
    if(transitionSI_idx==0)
    {
        ret.clear();
        ret.resize(12,0);
        leg_joint_targets_start.clear();
        leg_joint_targets_start.resize(12,0);
        leg_joint_targets_end.clear();
        leg_joint_targets_end.resize(12,0);
         


        /* 
        ret[0] = alljoints[KDeviceLists::L_LEG + KDeviceLists::HIP_YAW_PITCH];
        ret[1] = alljoints[KDeviceLists::L_LEG + KDeviceLists::HIP_ROLL];
        ret[2] = alljoints[KDeviceLists::L_LEG + KDeviceLists::HIP_PITCH];
        ret[3] = alljoints[KDeviceLists::L_LEG + KDeviceLists::KNEE_PITCH];
        ret[4] = alljoints[KDeviceLists::L_LEG + KDeviceLists::ANKLE_PITCH];
        ret[5] = alljoints[KDeviceLists::L_LEG + KDeviceLists::ANKLE_ROLL];




        ret[6] = alljoints[KDeviceLists::R_LEG + KDeviceLists::HIP_YAW_PITCH];
        ret[7] = alljoints[KDeviceLists::R_LEG + KDeviceLists::HIP_ROLL];
        ret[8] = alljoints[KDeviceLists::R_LEG + KDeviceLists::HIP_PITCH];
        ret[9] = alljoints[KDeviceLists::R_LEG + KDeviceLists::KNEE_PITCH];
        ret[10] = alljoints[KDeviceLists::R_LEG + KDeviceLists::ANKLE_PITCH];
        ret[11] = alljoints[KDeviceLists::R_LEG + KDeviceLists::ANKLE_ROLL];
        */
        /*
        armangles.setZero();
        armangles(0)=alljoints[KDeviceLists::L_ARM + KDeviceLists::SHOULDER_PITCH];
        armangles(1)=alljoints[KDeviceLists::L_ARM + KDeviceLists::SHOULDER_ROLL];
        armangles(2)=alljoints[KDeviceLists::L_ARM + KDeviceLists::ELBOW_YAW];
        armangles(3)=alljoints[KDeviceLists::L_ARM + KDeviceLists::ELBOW_ROLL];
        armangles(4)=alljoints[KDeviceLists::R_ARM + KDeviceLists::SHOULDER_PITCH];
        armangles(5)=alljoints[KDeviceLists::R_ARM + KDeviceLists::SHOULDER_ROLL];
        armangles(6)=alljoints[KDeviceLists::R_ARM + KDeviceLists::ELBOW_YAW];
        armangles(7)=alljoints[KDeviceLists::R_ARM + KDeviceLists::ELBOW_ROLL];
        */



        leg_joint_targets_start[0] = alljoints[KDeviceLists::L_LEG + KDeviceLists::HIP_YAW_PITCH];
        leg_joint_targets_start[1] = alljoints[KDeviceLists::L_LEG + KDeviceLists::HIP_ROLL];
        leg_joint_targets_start[2] = alljoints[KDeviceLists::L_LEG + KDeviceLists::HIP_PITCH];
        leg_joint_targets_start[3] = alljoints[KDeviceLists::L_LEG + KDeviceLists::KNEE_PITCH];
        leg_joint_targets_start[4] = alljoints[KDeviceLists::L_LEG + KDeviceLists::ANKLE_PITCH];
        leg_joint_targets_start[5] = alljoints[KDeviceLists::L_LEG + KDeviceLists::ANKLE_ROLL];
        leg_joint_targets_start[6] = alljoints[KDeviceLists::R_LEG + KDeviceLists::HIP_YAW_PITCH];
        leg_joint_targets_start[7] = alljoints[KDeviceLists::R_LEG + KDeviceLists::HIP_ROLL];
        leg_joint_targets_start[8] = alljoints[KDeviceLists::R_LEG + KDeviceLists::HIP_PITCH];
        leg_joint_targets_start[9] = alljoints[KDeviceLists::R_LEG + KDeviceLists::KNEE_PITCH];
        leg_joint_targets_start[10] = alljoints[KDeviceLists::R_LEG + KDeviceLists::ANKLE_PITCH];
        leg_joint_targets_start[11] = alljoints[KDeviceLists::R_LEG + KDeviceLists::ANKLE_ROLL];



    
        leg_joint_targets_end[0] = 4.19617e-05;
        leg_joint_targets_end[1] = -0.0152981;
        leg_joint_targets_end[2] = -0.454022;
        leg_joint_targets_end[3] = 1.18114;
        leg_joint_targets_end[4] = -0.722556;
        leg_joint_targets_end[5] = 0.0107799;
        leg_joint_targets_end[6] = 0.0;
        leg_joint_targets_end[7] = 0.0153821;
        leg_joint_targets_end[8] = -0.45564;
        leg_joint_targets_end[9] = 1.17969;
        leg_joint_targets_end[10] = -0.720938;
        leg_joint_targets_end[11] = -0.0199001;

        armangles_s.setZero();
        armangles_e.setZero();

        armangles.setZero();
        armangles_s(0)=alljoints[KDeviceLists::L_ARM + KDeviceLists::SHOULDER_PITCH];
        armangles_s(1)=alljoints[KDeviceLists::L_ARM + KDeviceLists::SHOULDER_ROLL];
        armangles_s(2)=alljoints[KDeviceLists::L_ARM + KDeviceLists::ELBOW_YAW];
        armangles_s(3)=alljoints[KDeviceLists::L_ARM + KDeviceLists::ELBOW_ROLL];
        armangles_s(4)=alljoints[KDeviceLists::R_ARM + KDeviceLists::SHOULDER_PITCH];
        armangles_s(5)=alljoints[KDeviceLists::R_ARM + KDeviceLists::SHOULDER_ROLL];
        armangles_s(6)=alljoints[KDeviceLists::R_ARM + KDeviceLists::ELBOW_YAW];
        armangles_s(7)=alljoints[KDeviceLists::R_ARM + KDeviceLists::ELBOW_ROLL];

        armangles_e(0)=1.55543;
        armangles_e(1)=0.271476;
        armangles_e(2)=-1.36377;
        armangles_e(3)=-0.29602;
        armangles_e(4)=1.54785;
        armangles_e(5)=-0.228608;
        armangles_e(6)=1.38056;
        armangles_e(7)=0.260822;

    }
    
        armangles(0)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(0), armangles_s(0),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(1)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(1), armangles_s(1),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(2)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(2), armangles_s(2),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(3)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(3), armangles_s(3),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(4)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(4), armangles_s(4),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(5)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(5), armangles_s(5),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(6)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(6), armangles_s(6),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        armangles(7)=interp.LinearInterpolation( (float) transitionSI_idx, armangles_e(7), armangles_s(7),NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);



        ret[0]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[0], leg_joint_targets_start[0],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[1]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[1], leg_joint_targets_start[1],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[2]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[2], leg_joint_targets_start[2],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[3]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[3], leg_joint_targets_start[3],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[4]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[4], leg_joint_targets_start[4],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[5]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[5], leg_joint_targets_start[5],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[6]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[6], leg_joint_targets_start[6],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[7]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[7], leg_joint_targets_start[7],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[8]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[8], leg_joint_targets_start[8],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[9]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[9], leg_joint_targets_start[9],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[10]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[10], leg_joint_targets_start[10],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);
        ret[11]=interp.LinearInterpolation( (float) transitionSI_idx, leg_joint_targets_end[11], leg_joint_targets_start[11],NaoRobot.getWalkParameter(transitionSI_instructions)-1.0);




    transitionSI_idx++;
    if(transitionSI_idx>NaoRobot.getWalkParameter(transitionSI_instructions)-1.0)
        isEstimationLoopInitialized=true;

    return ret;
}



std::vector<float> WalkEngine::Calculate_IK()
{
    /** --------------------------------------------------------------- **/
    
    /** CHECK FOR EARLY GROUND CONTACT **/
    
    /** --------------------------------------------------------------- **/
    /** Targets for leg feet Transformations **/
    Til=getTransformation(NaoFeetEngine.getFootXYTheta(LeftFoot), NaoFeetEngine.getFootZ(LeftFoot));
    Tir=getTransformation(NaoFeetEngine.getFootXYTheta(RightFoot), NaoFeetEngine.getFootZ(RightFoot));
    /** ------------------------------------------------------------------------ **/
    /** ------------------------------------------------------------------------ **/
    Til_eig = transformKMatToEigen(Til);
    Tir_eig = transformKMatToEigen(Tir);
    forceLw = Til_eig.linear()*forceL;
    forceRw = Tir_eig.linear()*forceR;

   
    rightEarlyContact=false;
    leftEarlyContact=false;
    rightLateContact=false;
    leftLateContact=false;


    if(GroundContact)
    {
        if (forceRw(2)>NaoRobot.getWalkParameter(Early_Contact_threshold) && !right_support_id && !double_support_id)
        {
            rightEarlyContact=true;
        }
  
        if (forceLw(2)>NaoRobot.getWalkParameter(Early_Contact_threshold) && right_support_id && !double_support_id)
        {
            leftEarlyContact=true;
        }
  
        
        if (forceRw(2)< 2.0* NaoRobot.getWalkParameter(Ground_Contact_threshold) && double_support_id)
        {
            rightLateContact=true;
        }
        
        if (forceLw(2)< 2.0 * NaoRobot.getWalkParameter(Ground_Contact_threshold) && double_support_id)
        {
            leftLateContact=true;
        }
    }

    /** --------------------------------------------------------------- **/
    /** --------------------------------------------------------------- **/
    /** --------------------------------------------------------------- **/
    
    /** FEET TRAJECTORIES **/
    NaoFeetEngine.MotionPlan(ci.target, currentstep, ci.steps, right_support_id, double_support_id, rightEarlyContact, leftEarlyContact,rightLateContact, leftLateContact);
    /** --------------------------------------------------------------- **/

    /** ------------------------------------------------------------------------ **/
    /** ------------------------------------------------------------------------ **/
    /** SET DESIRED COMI and Feet Homogeneous Transformations **/
    /** ------------------------------------------------------------------------ **/
    
    //desired=KVecDouble3(NaoLIPM.comx_d,NaoLIPM.comy_d,NaoRobot.getWalkParameter(ComZ)).scalar_mult(1000);
    desired=KVecDouble3(CoM_c(0),CoM_c(1),CoM_c(2)).scalar_mult(1000);

    
     //if(startup==NaoRobot.getWalkParameter(Init_instructions) && double_support_id)
     //desired(2) -= (CoM_ac.Control(forceR(2)+forceL(2)))*1000.0;
     //else if(!CoM_ac.resetted)
     //CoM_ac.reset();
     
    /** Targets for leg feet Transformations **/
    Til=getTransformation(NaoFeetEngine.getFootXYTheta(LeftFoot), NaoFeetEngine.getFootZ(LeftFoot));
    Tir=getTransformation(NaoFeetEngine.getFootXYTheta(RightFoot), NaoFeetEngine.getFootZ(RightFoot));
    /** ------------------------------------------------------------------------ **/
    /** ------------------------------------------------------------------------ **/
    //Til_eig = transformKMatToEigen(Til);
    //Tir_eig = transformKMatToEigen(Tir);

    if(startup==0)
        CoMs=Tip.transform(nkin.calculateCenterOfMass());
    
    if(startup<NaoRobot.getWalkParameter(Init_instructions)){
        desired(0)=interp.LinearInterpolation( (float) startup,desired(0), CoMs(0), NaoRobot.getWalkParameter(Init_instructions)-1.0);
        desired(1)=interp.LinearInterpolation( (float) startup,desired(1), CoMs(1), NaoRobot.getWalkParameter(Init_instructions)-1.0);
        desired(2)=interp.LinearInterpolation( (float) startup,NaoRobot.getWalkParameter(ComZ)*(1000.0), CoMs(2), NaoRobot.getWalkParameter(Init_instructions)-1.0);
    }
 
    /** Computing the COM error in the Inertial Frame of Reference **/
    com_error=desired;
    com_error-=CoMi;

    NAOKinematics::kmatTable Tpprimei; /** Transformation of target pelvis p' **/
    
    /**     First generate Tipprime and then invert           **/
    /** Generate Transformation from inertial to Pelvis' (next pelvis)**/
    
    /** Fix rotation first, using yawpitchroll coupling **/
    KMath::KMat::transformations::makeRotationZYX(Tpprimei,(double) anglemean(NaoFeetEngine.getFootTheta(LeftFoot),NaoFeetEngine.getFootTheta(RightFoot)),0.000 , 0.000);
    Tpprimei.setTranslation(com_error+Tip.getTranslation());
    /** Generate Transformation from Pelvis' to Inertial **/
    Tpprimei.fast_invert();
    /** Targets to Inverse Kinematics **/
    Tpprimel=Tpprimei*Til;
    Tpprimer=Tpprimei*Tir;
    /** Inverse Kinematics Procedure - Generate Desired Joint Angles **/
    resultL = nkin.inverseLeftLeg(Tpprimel);
    resultR = nkin.inverseRightLeg(Tpprimer);
    
    /** Set Leg Kinematics Chains **/
    ret.clear();
    
    if (!resultL.empty())
    {
        if (!resultR.empty())
        {
            ret = resultL.at(0);
            ret.insert(ret.end(), resultR.at(0).begin(), resultR.at(0).end());
            ret[0]=ret[6];
        }
        else
            std::cerr << "Right Leg EMPTY VECTOR " << std::endl;
    }
    else
        std::cerr << "Left Leg EMPTY VECTOR " << std::endl;

    /** Ankle Correction with PID Control **/
        NaoPosture.torsoStabilizer(Angle(0), Angle(1), 0.0, 0.0, ret[2], ret[8], ret[1], ret[7]);
        ret[2]=NaoPosture.lhip_Pitch;
        ret[8]=NaoPosture.rhip_Pitch;
        ret[1]=NaoPosture.lhip_Roll;
        ret[7]=NaoPosture.rhip_Roll;
    if(GroundContact)
    {

        zmpDist.computeDistribution(Vector3f(NaoLIPM.vrpx_d,NaoLIPM.vrpy_d,0.0), Vector3f(copi(0),copi(1),0.0),  forceLw,  forceRw,
                                 Vector3f(nipmEKF->fX,nipmEKF->fY,nipmEKF->fZ), Til_eig.translation(),  Tir_eig.translation(), Til_eig, Tir_eig,  right_support_id,  double_support_id);

        NaoPosture.ankleStabilizer(zmpDist.tauld, zmpDist.taurd, zmpDist.taul, zmpDist.taur,
                                   ret[4], ret[10], ret[5], ret[11]);


        if(double_support_id){
            ret[4]=NaoPosture.lankle_Pitch;
            ret[10]=NaoPosture.rankle_Pitch;
            ret[5]=NaoPosture.lankle_Roll;
            ret[11]=NaoPosture.rankle_Roll;

           
            NaoPosture.kneeStabilizer(zmpDist.fl(2), zmpDist.fr(2),zmpDist.fld(2), zmpDist.frd(2),  ret[3],  ret[9]);
            ret[3]=NaoPosture.lknee_Pitch;
            ret[9]=NaoPosture.rknee_Pitch;
        }
        else if(right_support_id)
        {
            ret[10]=NaoPosture.rankle_Pitch;
            ret[11]=NaoPosture.rankle_Roll;
            NaoPosture.dKnee_Pitch=0.0;
        }
        else
        {
            ret[4]=NaoPosture.lankle_Pitch;
            ret[5]=NaoPosture.lankle_Roll;
            NaoPosture.dKnee_Pitch=0.0;
        }
    }
    else
    {
        NaoPosture.dLAnkle_Roll = 0.0;
        NaoPosture.dRAnkle_Roll = 0.0;
        NaoPosture.dLAnkle_Pitch = 0.0;
        NaoPosture.dRAnkle_Pitch = 0.0;
    }
    

    /** Arm Joint Angles Computation **/
    
    armangles_temp.setZero();
    armangles.setZero();
    
    KVecDouble3 armt;
    armt=Tpprimel.getTranslation();
    armt.scalar_mult(1.0);
    //armt.prettyPrint();
    armangles_temp(2)=asin((-armt(0)+NaoRobot.getWalkParameter(HX)*1000)/(UpperArmLength*1.0) )+M_PI_2;
    armangles_temp(1)=asin((armt(1)+85-ShoulderOffsetY)/(UpperArmLength*1.25));
    armt=Tpprimer.getTranslation();
    armt.scalar_mult(1.0);
    //armt.prettyPrint();
    armangles_temp(0)=asin((-armt(0)+NaoRobot.getWalkParameter(HX)*1000)/(UpperArmLength*1.0) )+M_PI_2;
    armangles_temp(3)=asin((-armt(1)+85-ShoulderOffsetY)/(UpperArmLength*1.25) );
    //armangles.prettyPrint();

    if(startup==0)
    {
        armangles_s.setZero();
        armangles_s(0)=alljoints[KDeviceLists::L_ARM + KDeviceLists::SHOULDER_PITCH];
        armangles_s(1)=alljoints[KDeviceLists::L_ARM + KDeviceLists::SHOULDER_ROLL];
        armangles_s(2)=alljoints[KDeviceLists::L_ARM + KDeviceLists::ELBOW_YAW];
        armangles_s(3)=alljoints[KDeviceLists::L_ARM + KDeviceLists::ELBOW_ROLL];
        armangles_s(4)=alljoints[KDeviceLists::R_ARM + KDeviceLists::SHOULDER_PITCH];
        armangles_s(5)=alljoints[KDeviceLists::R_ARM + KDeviceLists::SHOULDER_ROLL];
        armangles_s(6)=alljoints[KDeviceLists::R_ARM + KDeviceLists::ELBOW_YAW];
        armangles_s(7)=alljoints[KDeviceLists::R_ARM + KDeviceLists::ELBOW_ROLL];
    }

    armangles(0)=armangles_temp(0);
    armangles(1)=armangles_temp(1);
    armangles(2)=armangles_s(2);
    armangles(3)=-armangles_temp(1);
    armangles(4)=armangles_temp(2);
    armangles(5)=-armangles_temp(3);
    armangles(6)=armangles_s(6);
    armangles(7)=armangles_temp(3);
    
    if(startup<NaoRobot.getWalkParameter(Init_instructions))
    {
        armangles(0)=interp.LinearInterpolation( (float) startup,armangles(0), armangles_s(0),NaoRobot.getWalkParameter(Init_instructions)-1.0);
        armangles(1)=interp.LinearInterpolation( (float) startup,armangles(1), armangles_s(1),NaoRobot.getWalkParameter(Init_instructions)-1.0);
        armangles(3)=interp.LinearInterpolation( (float) startup,armangles(3), armangles_s(3),NaoRobot.getWalkParameter(Init_instructions)-1.0);
        armangles(4)=interp.LinearInterpolation( (float) startup,armangles(4), armangles_s(4),NaoRobot.getWalkParameter(Init_instructions)-1.0);
        armangles(5)=interp.LinearInterpolation( (float) startup,armangles(5), armangles_s(5),NaoRobot.getWalkParameter(Init_instructions)-1.0);
        armangles(7)=interp.LinearInterpolation( (float) startup,armangles(7), armangles_s(7),NaoRobot.getWalkParameter(Init_instructions)-1.0);
        startup++;
    }

    currentstep++;
    return ret;
}

/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* ---------------        Complementary Functions to WalkEngine               --------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/
/* -----------------------------------------------------------------------------------------------------*/




/**
 *  This function sets the FSR values
 **/
void WalkEngine::setFSR(Vector4f l,Vector4f r, Vector3f cl, Vector3f cr, float wl, float wr )
{
    fsrl=l;
    fsrr=r;

    if(!FSRDataInit)
        FSRDataInit = true;

    if(wl*NaoRobot.getWalkParameter(g)>NaoRobot.getWalkParameter(LegLowThres))
    {
        forceL = Vector3f(0,0,leftGRF_LPF->filter(wl*NaoRobot.getWalkParameter(g)));

        copl(0) = coplx_LPF->filter(cl(0));
        copl(1) = coply_LPF->filter(cl(1));
        copl(2) = cl(2);
        copl_cropped(0) = crop(copl(0),0.07025,-0.02965);
        copl_cropped(1) = crop(copl(1),0.0299,-0.0191);
        copl_cropped(2) = copl(2);
        noContactLeft=false;

    }
    else
    {
        forceL = Vector3f(0,0,leftGRF_LPF->filter(0.0));
        copl(0) = coplx_LPF->filter(0.0);
        copl(1) = coply_LPF->filter(0.0);
        copl(2) = cl(2);
        copl_cropped.setZero();
        noContactLeft=true;
    }
    if(wr*NaoRobot.getWalkParameter(g)>NaoRobot.getWalkParameter(LegLowThres))
    {

        forceR = Vector3f(0,0,rightGRF_LPF->filter(wr*NaoRobot.getWalkParameter(g)));
        copr(0) = coprx_LPF->filter(cr(0));
        copr(1) = copry_LPF->filter(cr(1));
        copr(2) = cr(2);

        copr_cropped(0) = crop(copr(0),0.07025,-0.02965);
        copr_cropped(1) = crop(copr(1),0.0191, -0.0299);
        copr_cropped(2) = copr(2);
        noContactRight=false;
    }
    else
    {
        forceR = Vector3f(0,0,rightGRF_LPF->filter(0.0));
        copr(0) = coprx_LPF->filter(0.0);
        copr(1) = copry_LPF->filter(0.0);
        copr(2) = cr(2);
        copr_cropped.setZero();
        noContactRight=true;
    }
}

void WalkEngine::AddVelocityInstruction(KVecFloat3 vel)
{
    velocityQ.push_back(vel);
}
void WalkEngine::setJoints(std::vector<float> joints)
{
    if(!JointDataInit)
        JointDataInit = true;
    
    // for (int j = 0, i = 0; i < KDeviceLists::NUMOFJOINTS; i++, j++)
    // {
        
    //     JointKF[j].Filter(joints[i]);
    //     // std::cout<<"Joint "<<i<<" Val: "<<JointKF[j].JointPosition<<std::endl;
    //     alljoints[j] =JointKF[j].JointPosition;
        
    // }
    //alljoints[KDeviceLists::R_LEG+KDeviceLists::HIP_YAW_PITCH]=alljoints[KDeviceLists::L_LEG+KDeviceLists::HIP_YAW_PITCH];
    alljoints = joints;
    nkin.setJoints(joints);
    
}


/**
 * This Function returns the position of the Center of Pressure (CoP) as measured by
 * the Foot Sensitive Resistors (FSRs) in the Inertial Frame of Reference
 **/
Vector3f WalkEngine::getCoP()
{
    
    Vector3f res;

    res.setZero();
    Vector3f  cops, copsprime, fs, fsprime, cops_cropped, copsprime_cropped;
    float  weights, weightsprime;
    fs.setZero();
    fsprime.setZero();
    copsprime.setZero();
    cops.setZero();
    cops_cropped.setZero();
    copsprime_cropped.setZero();

    if(noContactRight && noContactLeft)
         return res;
    
    if (supportleg == KDeviceLists::SUPPORT_LEG_RIGHT) {
        cops = copr;
        cops_cropped = copr_cropped;

        fs =  forceR;
        weights = forceR(2)/NaoRobot.getWalkParameter(g);
        
        copsprime = copl;
        copsprime_cropped = copl_cropped;
        fsprime = forceL;
        weightsprime =  forceL(2)/NaoRobot.getWalkParameter(g);
    }
    else
    {
        cops = copl;
        cops_cropped = copl_cropped;
        fs =  forceL;
        weights = forceL(2)/NaoRobot.getWalkParameter(g);

        fsprime = forceR;
        copsprime = copr;
        copsprime_cropped = copr_cropped;
        weightsprime = forceR(2)/NaoRobot.getWalkParameter(g);
    }
 
    if (fsprime(2) < NaoRobot.getWalkParameter(LegLowThres) )
    {
        weightsprime = 0.00;
        fsprime.setZero();
        copsprime.setZero();
        copsprime_cropped.setZero();
    }
 
    if (fs(2)<NaoRobot.getWalkParameter(LegLowThres))
    {
        weights = 0.00;
        fs.setZero();
        cops.setZero();
        cops_cropped.setZero();
    }
    if(weightsprime+weights>0.0)
    {
    /** Compute the CoP wrt the Support Foot Frame **/
    //copi = cops.scalar_mult(weights)
    //                        + Tssprime.transform(copsprime).scalar_mult(weightsprime);
    //copi.scalar_mult(1.0 / (weights + weightsprime));
    res = cops * weights + Tssprime_eig * copsprime  * weightsprime;
    res /= (weights+weightsprime);
    copsrel = res;
    res = Tis_eig*res; 

    copi_cropped = cops_cropped * weights + Tssprime_eig * copsprime_cropped  * weightsprime;
    copi_cropped /= (weights+weightsprime);

    fis = fs  +  Tssprime_eig.linear()* fsprime;
    fis = Tis_eig.linear()*fis;
    }
    return res;
}

float WalkEngine::crop(float v_, float max_v, float min_v)
{
     return fmax(min_v, fmin(v_,max_v));

}


void WalkEngine::setIMU(Vector3f acc, Vector3f gyro , Vector3f angle)
{
    if(!IMUDataInit)
        IMUDataInit = true;

    Acc=acc;
    Gyro=gyro;
    Angle=angle;
}



/*
 void  WalkEngine::determineLegContact() {
 
	if (!right_support_id)
	{
 supportleg = KDeviceLists::SUPPORT_LEG_LEFT;
 support_foot_id = "LLeg";
 swing_foot_id= "RLeg";
 chainsupport = KDeviceLists::CHAIN_L_LEG;
 chainswing = KDeviceLists::CHAIN_R_LEG;
	}
	else
	{
 supportleg = KDeviceLists::SUPPORT_LEG_RIGHT;
 support_foot_id= "RLeg";
 swing_foot_id= "LLeg";
 chainsupport = KDeviceLists::CHAIN_R_LEG;
 chainswing = KDeviceLists::CHAIN_L_LEG;
	}
 }
 */

