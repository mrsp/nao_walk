#include "FeetEngine.h"


FeetEngine::FeetEngine(RobotParameters &robot):NaoRobot(robot)
{
    default_params();
    std::cout<<"FeetEngine Module Initialized Successfully"<<std::endl;
}
void FeetEngine::reset()
{
    default_params();
    std::cout<<"FeetEngine Module Reseted Successfully"<<std::endl;
}



void FeetEngine::default_params()
{
    StepZ_ = NaoRobot.getWalkParameter(StepZ);
    FootL.zero();
    FootR.zero();
    startR.zero();
    startL.zero();
    planR.zero();
    planL.zero();
    startLz=0.000;
    startRz=0.000;
    planLz=0.000;
    planRz=0.000;
    FootLz=0.00;
    FootRz=0.00;
}

void FeetEngine::setFootStartXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight)
{
    if (isRight)
    {
        startR(0)=xytheta(0);
        startR(1)=xytheta(1);
        startR(2)=xytheta(2);
    }
    else
    {
        startL(0)=xytheta(0);
        startL(1)=xytheta(1);
        startL(2)=xytheta(2);
    }
}

void FeetEngine::setFootDestXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight)
{
    if (isRight)
    {
        planR(0)=xytheta(0);
        planR(1)=xytheta(1);
        planR(2)=xytheta(2);
    }
    else
    {
        planL(0)=xytheta(0);
        planL(1)=xytheta(1);
        planL(2)=xytheta(2);
    }
}

void FeetEngine::setFootXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight)
{
    if (isRight)
    {
        FootR(0)=xytheta(0);
        FootR(1)=xytheta(1);
        FootR(2)=xytheta(2);
    }
    else
    {
        FootL(0)=xytheta(0);
        FootL(1)=xytheta(1);
        FootL(2)=xytheta(2);
    }
}

void FeetEngine::setFootZ(float z, bool isRight)
{
    if (isRight)
        FootRz=z;
    else
        FootLz=z;
}

void FeetEngine::setFootStartZ(float z, bool isRight)
{
    if (isRight)
        startRz=z;
    else
        startLz=z;
}

void FeetEngine::setFootDestZ(float z, bool isRight)
{
    if (isRight)
        planRz=z;
    else
        planLz=z;
}



KVecFloat3 FeetEngine::getFootDestXYTheta(bool isRight)
{
    KVecFloat3 res;
    if (isRight)
    {
        res(0)=planR(0);
        res(1)=planR(1);
        res(2)=planR(2);
    }
    else
    {
        res(0)=planL(0);
        res(1)=planL(1);
        res(2)=planL(2);
    }

    return res;

}

float FeetEngine::getFootDestTheta(bool isRight)
{
    float res;
    if (isRight)
        res=planR(2);
    else
        res=planL(2);


    return res;

}



float FeetEngine::getFootDestZ(bool isRight)
{
    float res;
    if (isRight)
        res=planRz;
    else
        res=planLz;


    return res;

}

float FeetEngine::getFootTheta(bool isRight)
{
    float res;
    if (isRight)
        res=FootR(2);
    else
        res=FootL(2);


    return res;

}


float FeetEngine::getFootZ(bool isRight)
{
    float res;
    if (isRight)
        res=FootRz;
    else
        res=FootLz;

    return res;

}

KVecFloat3 FeetEngine::getFootXYTheta(bool isRight)
{
    KVecFloat3 res;
    if (isRight)
        res=FootR;
    else
        res=FootL;

    return res;

}




void FeetEngine::MotionPlan(KVecFloat3 target, unsigned step, unsigned totalsteps, bool right_support, bool double_support, bool RightEarlyContact, bool LeftEarlyContact, bool RightLateContact, bool LeftLateContact)
{



    //Generate Feet Trajectories while swing
    if(!right_support && !double_support)
    {
      
        if (!RightEarlyContact)
        {

            /** Right Foot Interpolation **/
            float diff=KMath::anglediff2(target(2),startR(2));

            //if (ci.state==KICK)
            //dr(0)=planForwardKick( (float)currentstep, ci.target(0), startR(0), ci.steps-1);
            //else
            FootR(0)=interp.LinearInterpolation((float) step, target(0), startR(0), totalsteps-1.0);

            FootR(1)=interp.LinearInterpolation((float) step, target(1), startR(1), totalsteps-1.0);


            FootR(2)=startR(2)+interp.LinearInterpolation((float) step, diff, 0.0, totalsteps-1.0);

            // if(!RightLateContact)
            // {
                FootRz=interp.CubicSplineInterpolation( (float) step, 0.000, StepZ_/2.0, StepZ_, StepZ_/4.0,0.000,totalsteps-1.0);
                //FootRz=interp.planFeetTrajectoryZ((float) step, NaoRobot.getWalkParameter(StepZ),0.000, totalsteps);
               // FootRz=interp.BezierZ((float) step, NaoRobot.getWalkParameter(StepZ), totalsteps-1.0);
            // }
            // else if(FootRz > -0.03)
            // {
            //     FootRz -= 0.005;
            // }


        }
  
    }
    else if(right_support && !double_support)
    {
        if (!LeftEarlyContact)
        {
            /** Left Foot Interpolation **/
            float diff=KMath::anglediff2(target(2),startL(2));
            
            FootL(0)=interp.LinearInterpolation((float) step, target(0), startL(0), totalsteps-1.0);

            FootL(1)=interp.LinearInterpolation((float) step, target(1), startL(1), totalsteps-1.0);

            FootL(2)=startL(2)+interp.LinearInterpolation( (float) step, diff, 0.0, totalsteps-1.0);

            // if(!LeftLateContact)
            // {
                FootLz=interp.CubicSplineInterpolation((float) step, 0.000, StepZ_/2.0, StepZ_, StepZ_/4.0,0.000,totalsteps-1.0);
               // FootLz=interp.planFeetTrajectoryZ((float) step, NaoRobot.getWalkParameter(StepZ),0.000, totalsteps);
                // FootLz=interp.BezierZ((float) step, NaoRobot.getWalkParameter(StepZ), totalsteps-1.0);

            // }
            // else if(FootLz > -0.03)
            // {
            //     FootLz -= 0.005;
            // }

        }
    }


}









