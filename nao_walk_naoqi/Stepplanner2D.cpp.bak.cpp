#include"Stepplanner2D.h"



Stepplanner2D::Stepplanner2D(RobotParameters robot_):robot(robot_)
{
        tempV.zero();
        targetXY.zero();
        pivotXY.zero();
        centerXY.zero();
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
        cout<<"2D Stepplanner: Initialized Successfully"<<endl;
}

WalkInstruction Stepplanner2D::planStep2D(KVecFloat3 v, SupportInstruction si)
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
    
        
        if(v(0)==v_(0) && v(1)==v_(1) and v(2)==v_(2) && v(0)==0 && v(1)==0 && v(2)==0)
        {
            cmd = STAND;
        }
        else
        {
            cmd = WALK;
        }
        


        if(si.targetSupport == KDeviceLists::SUPPORT_LEG_LEFT)
        {
            h = KVecFloat2(0.0, -2.0*robot.getWalkParameter(H0));
            c = KVecFloat2(-robot.getWalkParameter(HX),-2.0*robot.getWalkParameter(H0));
        }
        else
        {
            h = KVecFloat2(0.0, 2.0*robot.getWalkParameter(H0));
            c = KVecFloat2(-robot.getWalkParameter(HX), 2.0*robot.getWalkParameter(H0));
        }

        support_foot_x = si.pose(0);
        support_foot_y = si.pose(1);
        support_foot_orientation = si.pose(2);

        KMath::KMat::transformations::makeRotation(rot,support_foot_orientation);
        h = rot * h;
        c = rot * c;

        //Generate the pivot point
        pivotXY = KVecFloat2(support_foot_x + h(0)/2.0,support_foot_y + h(1)/2.0);

        targetXY = KVecFloat2(h(0)/2.0,h(1)/2.0);
        centerXY = KVecFloat2(c(0) - h(0)/2.0,c(1) -h(1)/2.0);
        targetTheta = support_foot_orientation;

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
        targetXY += pivotXY;
        centerXY = rot * centerXY;
        centerXY += pivotXY;

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

        //Output must be a Walk Instruction
        ci.target(0) = targetXY(0);
        ci.target(1) = targetXY(1);
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
        cout<<"Stepplanner2D: Step Planned"<<endl;
        return ci;
}
