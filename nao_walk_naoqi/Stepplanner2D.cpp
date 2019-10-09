#include"Stepplanner2D.h"
Stepplanner2D::Stepplanner2D(RobotParameters robot_):robot(robot_),stepAnkleQ(robot_.getWalkParameter(StepPlanSize)), velocityQ(robot_.getWalkParameter(StepPlanSize))
{
        tempV.zero();
        targetXY.zero();
        pivotXY.zero();
        centerXY.zero();
        MaxStep.zero();
        MinStep.zero();
        c.zero();
        h.zero();
        rot.zero();
        step_id = 0;
        cmd = 0;
        support_foot_x = 0.0;
        support_foot_y = 0.0;
        support_foot_orientation = 0.0;
        v_.zero();
        eps=1.0e-2;
        planAvailable = false;
        cout<<"2D Stepplanner: Initialized Successfully"<<endl;
}

void Stepplanner2D::plan(WalkInstruction si)
{
    unsigned int jjj = 0;
    while(velocityQ.size()>0)
    {
        WalkInstruction pi;
        if(jjj==0)
        {
            pi = planStep2D(velocityQ.front(), si);
        }
        else
        {
            pi = planStep2D(velocityQ.front(), pi);
        }
        stepAnkleQ.push_back(pi);
        velocityQ.pop_front();
        jjj++;
    }
    planAvailable = true;
}

void Stepplanner2D::emptyPlan()
{
    while(velocityQ.size()>0)
        velocityQ.pop_front();
    
    while(stepAnkleQ.size()>0)
        stepAnkleQ.pop_front();

    planAvailable = false;

}


WalkInstruction Stepplanner2D::planStep2D(KVecFloat3 v, WalkInstruction si)
{
        
	WalkInstruction ci;

        if (abs(v(0))<eps && abs(v(1))<eps && abs(v(2))<eps)
        {
            v.zero();
        }

        //Crop the Velocity in feasible limits
        if(v(0)>0.75)
            v(0) = 0.75;
        if(v(0)<-1.00)
            v(0) = -1.00;

        if(v(1)>1.00)
            v(1) = 1.00;
        if(v(1)<-1.00)
            v(1) = -1.00;

        if(v(2)>0.80)
            v(2) = 0.80;
        if(v(2)<-0.80)
            v(2) = -0.80;
    

        //Previous Swing Foot Becomes Support
        support_foot_x = si.target(0);
        support_foot_y = si.target(1);
        support_foot_orientation = si.target(2);
        //Support Leg exchange
        if(si.targetSupport ==  KDeviceLists::SUPPORT_LEG_LEFT)
        {
            si.targetSupport = KDeviceLists::SUPPORT_LEG_RIGHT;
        }
        else
        {
            si.targetSupport =  KDeviceLists::SUPPORT_LEG_LEFT;
        }
        
        if(si.targetSupport == KDeviceLists::SUPPORT_LEG_LEFT)
        {
            h = KVecFloat2(0.0, -2.0*robot.getWalkParameter(H0));
        }
        else
        {
            h = KVecFloat2(0.0, 2.0*robot.getWalkParameter(H0));
        }



        KMath::KMat::transformations::makeRotation(rot,support_foot_orientation);
        h = rot * h;

        targetXY = h;
        targetTheta = support_foot_orientation;

        if(v(0)==v_(0) && v(1)==v_(1) and v(2)==v_(2) && v(0)==0 && v(1)==0 && v(2)==0)
        {
            cmd = STAND;


        }
        else
        {
            cmd = WALK;
        }
        



        rot.zero();
        if(v(2)>0)
        {
            if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            {
                KMath::KMat::transformations::makeRotation(rot,v(2)*robot.getWalkParameter(MaxStepTheta));
            }
            else
            {
                KMath::KMat::transformations::makeRotation(rot,-v(2)*robot.getWalkParameter(MinStepTheta));
            }
        }
        else
        {
            if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            {
                KMath::KMat::transformations::makeRotation(rot,-v(2)*robot.getWalkParameter(MinStepTheta));
            }
            else
            {
                 KMath::KMat::transformations::makeRotation(rot,v(2) * robot.getWalkParameter(MaxStepTheta));
            }
        }
        targetXY = rot * targetXY;
        
        //Generate Constraints
        tempV.zero();
        if(v(0) > 0.00)
        {
            tempV(0) = v(0) * robot.getWalkParameter(MaxStepX);
        }
        else
        {
            tempV(0) = -v(0) * robot.getWalkParameter(MinStepX);
        }
       

        if(v(1) > 0.00)
        {
            if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            {
                tempV(1) = v(1) * robot.getWalkParameter(MaxStepY);
            }
            else
            {
                tempV(1) = -v(1) * robot.getWalkParameter(MinStepY) * 0.0;
            }
        }
        else
        {
            if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            {
                tempV(1) = -v(1) * robot.getWalkParameter(MinStepY) * 0.0;
            }
            else
            {
                tempV(1) = v(1) * robot.getWalkParameter(MaxStepY);
            }
        }

        rot.zero();
        KMath::KMat::transformations::makeRotation(rot,support_foot_orientation);
        tempV = rot * tempV;
        targetXY += tempV;

        if(v(2) > 0.00)
        {
            if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            {
                targetTheta += v(2) * robot.getWalkParameter(MaxStepTheta);
            }
            else
            {
                targetTheta -= v(2) * robot.getWalkParameter(MinStepTheta);
            }
        }
        else
        {
            if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
            {
                targetTheta -= v(2) * robot.getWalkParameter(MinStepTheta);
            }
            else
            {
                targetTheta += v(2) * robot.getWalkParameter(MaxStepTheta);
            }
        }

        //Fix Constraints
        MaxStep(0) = robot.getWalkParameter(MaxStepX);
        MinStep(0) = robot.getWalkParameter(MinStepX);

        if(si.targetSupport == KDeviceLists::SUPPORT_LEG_RIGHT)
        {
            MaxStep(1) = robot.getWalkParameter(MaxStepY);
            MinStep(1) = robot.getWalkParameter(MinStepY);

        }
        else
        {
            MaxStep(1) = -robot.getWalkParameter(MinStepY);
            MinStep(1) = -robot.getWalkParameter(MaxStepY);
        }
        


        tempV = rot.transp() * targetXY;
        

        tempV(0) = cropStep(tempV(0),MaxStep(0),MinStep(0));
        tempV(1) = cropStep(tempV(1),MaxStep(1),MinStep(1));
      
        
        targetXY = rot * tempV;

      
        //Output must be a Walk Instruction
        ci.target(0) = targetXY(0) + support_foot_x;
        ci.target(1) = targetXY(1) + support_foot_y;
        ci.target(2) = targetTheta;
        ci.targetSupport = si.targetSupport;
        ci.targetZMP = si.targetSupport;

       
        if(cmd == STAND)
        {
            ci.targetZMP = KDeviceLists::SUPPORT_LEG_BOTH;
        }
        
        ci.steps = robot.getWalkParameter(SS_instructions);
        step_id = si.step_id + 1;
	    ci.step_id = step_id;
        v_ = v;
        return ci;
}
