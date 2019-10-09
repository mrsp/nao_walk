#include "ZMPDistributor.h"
/* alpha = 1.0 -> right */

ZMPDistributor::ZMPDistributor(RobotParameters &robot_):robot(robot_), LFoot(robot_), RFoot(robot_)
{
    frd.setZero();
    fld.setZero();
    tauld.setZero();
    taurd.setZero();
    tau0.setZero();
    taul.setZero();
    taur.setZero();
    a = 0.000;
    maxForceReadingL = robot.getWalkParameter(mass)*robot.getWalkParameter(g);
    maxForceReadingR = robot.getWalkParameter(mass)*robot.getWalkParameter(g);


    std::cout<<"ZMP Distributor Initialized Successfully"<<std::endl;


}


void ZMPDistributor::computeDistribution(Vector3f pzmp_d, Vector3f pzmp, Vector3f fl_, Vector3f fr_, Vector3f fext, Vector3f LfootA, Vector3f RfootA, Affine3f Til, Affine3f Tir, bool right_support, bool double_support)
{
    LFoot.setPolygon(LfootA,Til);
    RFoot.setPolygon(RfootA,Tir);
    taul.setZero();
    taur.setZero();
    tauld.setZero();
    taurd.setZero();
    tau0.setZero();
    fld.setZero();
    frd.setZero();
    if(!double_support)
    {
        if(right_support)
        {
            frd(2) = maxForceReadingR;
            taurd = (RFoot.footA - pzmp_d).cross(frd);
            fr = fr_ + fext;     
            taur = (RFoot.footA - pzmp).cross(fr);       
   
        }
        else
        {
            fld(2) = maxForceReadingL;
            tauld =  (LFoot.footA - pzmp_d).cross(fld);
            fl = fl_ + fext;
            taul =  (LFoot.footA - pzmp).cross(fl+fext);
        }
    }
    else
    {

        //If ZMP is inside of one of the support polygons
        if(LFoot.pnpoly(pzmp_d(0),pzmp_d(1)))
        {
            a=0.0;
            fld(2) = maxForceReadingL;
            tauld =  (LFoot.footA - pzmp_d).cross(fld);

        }
        else if(RFoot.pnpoly(pzmp_d(0),pzmp_d(1)))
        {
            a=1.0;
            frd(2) = maxForceReadingR;
            taurd = (RFoot.footA - pzmp_d).cross(frd);          
        }
        else
        {
            pL = computePointLineIntersection(Vector2f(pzmp_d(0),pzmp_d(1)), Vector2f(LFoot.footRF(0),LFoot.footRF(1)), Vector2f(LFoot.footRH(0),LFoot.footRH(1)));
            pR = computePointLineIntersection(Vector2f(pzmp_d(0),pzmp_d(1)), Vector2f(RFoot.footLF(0),RFoot.footLF(1)), Vector2f(RFoot.footLH(0),RFoot.footLH(1)));
            pa = computePointLineIntersection(Vector2f(pzmp_d(0),pzmp_d(1)), pR, pL);
            
        
            //Compute Distribution parameter a
            a = (pa - pL).norm()/((pL-pR).norm());
            if(a>1.0)
                a= 1.0;


            frd(2) =  a * maxForceReadingR;
            fld(2) = (1.0-a) * maxForceReadingL;
            
            tau0 = -(RFoot.footA - pzmp_d).cross(frd) - (LFoot.footA - pzmp_d).cross(fld);
            taurd(1) = a * tau0(1);
            tauld(1) = (1.0-a) * tau0(1);

            if(tau0(0)<0)
            {
                taurd(0) = tau0(0);
                tauld(0) = 0.0;
            }
            else
            {
                taurd(0) = 0.0;
                tauld(0) = tau0(0);
            }
        }

        //If the measuerd ZMP is inside of one of the support polygons
        if(LFoot.pnpoly(pzmp(0),pzmp(1)))
        {
            a=0.0;
            fl = fl_ + fext;
            taul =  (LFoot.footA - pzmp).cross(fl);
        }
        else if(RFoot.pnpoly(pzmp(0),pzmp(1)))
        {
            a=1.0;
            fr = fr_ + fext;
            taur = (RFoot.footA - pzmp).cross(fr);       
        }
        else
        {
            pL = computePointLineIntersection(Vector2f(pzmp(0),pzmp(1)), Vector2f(LFoot.footRF(0),LFoot.footRF(1)), Vector2f(LFoot.footRH(0),LFoot.footRH(1)));
            pR = computePointLineIntersection(Vector2f(pzmp(0),pzmp(1)), Vector2f(RFoot.footLF(0),RFoot.footLF(1)), Vector2f(RFoot.footLH(0),RFoot.footLH(1)));
            pa = computePointLineIntersection(Vector2f(pzmp(0),pzmp(1)), pR, pL);
            
            //Compute Distribution parameter a
            a = (pa - pL).norm()/((pL-pR).norm());
            if(a>1.0)
                a= 1.0;
    

            fr=fr_+a*fext;
            fl=fl_+(1.0-a)*fext;
            tau0 = -(RFoot.footA - pzmp).cross(fr) - (LFoot.footA - pzmp).cross(fl);
            taur(1) = a * tau0(1);
            taul(1) = (1.0-a) * tau0(1);

            if(tau0(0)<0)
            {
                taur(0) = tau0(0);
                taul(0) = 0.0;
            }
            else
            {
                taur(0) = 0.0;
                taul(0) = tau0(0);
            }
        }
    }


}



