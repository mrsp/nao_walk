#include "CoMAdmittanceControl.h"

CoMAdmittanceControl::CoMAdmittanceControl(RobotParameters &robot):NaoRobot(robot)
{

    gain_x =  2.2;
    gain_y =  4.2;
    com_c.setZero();

    k_f(0,0) = gain_x;
    k_f(0,1) = 0.000;
    k_f(1,0) = 0.000;
    k_f(1,1) = gain_y;

    firstrun = true;

    rateX = 5.0;
    rateY = 5.0;
    ZMPXint.rate(rateX);
	ZMPXint.setZero();
    ZMPPXint.rate(rateX);
	ZMPPXint.setZero();
    ZMPYint.rate(rateY);
	ZMPYint.setZero();
    ZMPPYint.rate(rateY);
	ZMPPYint.setZero();
	int_zmpp.setZero();
	int_zmp.setZero();

    std::cout<<"CoM Admittance Controller Initialized Successfully"<<std::endl;
}




Vector2d CoMAdmittanceControl::Control(Vector2d com_d,Vector2d comd_d, Vector2d vrp_d, Vector2d vrp, Vector2d com)
{
    


    k_f(0,0) = gain_x;
    k_f(1,1) = gain_y;


   	if(firstrun)
	{
		firstrun = false;
		com_c = com_d;
 	}
 	else
	{
		//Force control for Trajectory Tracking
		if(k_f(0,0)>0)
		{	
			ZMPXint.add(vrp(0)-vrp_d(0),NaoRobot.getWalkParameter(Ts));
			int_zmp(0) = ZMPXint.eval();
			ZMPPXint.add(int_zmp(0),NaoRobot.getWalkParameter(Ts));
			int_zmpp(0) = ZMPPXint.eval();

		}

		if(k_f(1,1)>0)
		{
			ZMPYint.add(vrp(1)-vrp_d(1),NaoRobot.getWalkParameter(Ts));
			int_zmp(1) = ZMPYint.eval();
			ZMPPYint.add(int_zmp(1),NaoRobot.getWalkParameter(Ts));
			int_zmpp(1) = ZMPPYint.eval();
		}
	
		 com_c = com_d + k_f * int_zmpp;

	
	}


    return com_c;
    
    
}

