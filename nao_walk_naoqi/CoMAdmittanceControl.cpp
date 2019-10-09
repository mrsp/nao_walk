#include "CoMAdmittanceControl.h"

CoMAdmittanceControl::CoMAdmittanceControl(RobotParameters &robot):NaoRobot(robot)
{

	K_pitch = 0.0;
	K_roll =  0.0;
    gain_x =  1.25;
    gain_y =  2.25;
	//gain_x =  2.25;
    //gain_y =  4.25;
    com_c.setZero();

    k_f(0,0) = gain_x;
    k_f(0,1) = 0.000;
    k_f(1,0) = 0.000;
    k_f(1,1) = gain_y;
	zmp_delay = 5;
    firstrun = true;

    rateX = 7.5;
    rateY = 7.5;
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

Vector3d CoMAdmittanceControl::Control(Vector2d com_d, Vector2d zmp_d, Vector2d zmp, double roll, double pitch, double omega_roll, double omega_pitch, bool GroundContact)
{
	Vector3d com_out;

	com_out = Vector3d(com_d(0),com_d(1),NaoRobot.getWalkParameter(ComZ));


	if(GroundContact)
	{
		double capture_pitch, capture_roll;

		capture_pitch  = -(1.0+K_pitch/NaoRobot.getWalkParameter(omega))*NaoRobot.getWalkParameter(ComZ) * (pitch + 1.0/NaoRobot.getWalkParameter(omega) * omega_pitch);
		capture_roll  =  (1.0+K_roll/NaoRobot.getWalkParameter(omega))*NaoRobot.getWalkParameter(ComZ) * (roll + 1.0/NaoRobot.getWalkParameter(omega) * omega_roll);
		
		zmp_out = zmp_d  + Vector2d(capture_pitch, capture_roll); 


		zmp_d_buffer.push(zmp_out);
		

		if((zmp_d_buffer.size() > zmp_delay - 1) && !firstrun)
		{
			Vector2d temp;
			k_f(0,0) = gain_x;
			k_f(1,1) = gain_y;
			//Force control for Trajectory Tracking
			zmp_d_ = zmp_d_buffer.front();
			if(k_f(0,0)>0)
			{	
				ZMPXint.add(zmp(0)-zmp_d_(0),NaoRobot.getWalkParameter(Ts));
				int_zmp(0) = ZMPXint.eval();
				ZMPPXint.add(int_zmp(0),NaoRobot.getWalkParameter(Ts));
				int_zmpp(0) = ZMPPXint.eval();

			}

			if(k_f(1,1)>0)
			{
				ZMPYint.add(zmp(1)-zmp_d_(1),NaoRobot.getWalkParameter(Ts));
				int_zmp(1) = ZMPYint.eval();
				ZMPPYint.add(int_zmp(1),NaoRobot.getWalkParameter(Ts));
				int_zmpp(1) = ZMPPYint.eval();
			}

			temp = com_d + k_f * int_zmpp;
			com_out = Vector3d(temp(0),temp(1),NaoRobot.getWalkParameter(ComZ));	
			zmp_d_buffer.pop();	
		}
		else
		{
			firstrun = false;
		}
	}
	else
	{
			ZMPXint.setZero();
			ZMPYint.setZero();
			ZMPPXint.setZero();
			ZMPPYint.setZero();
	}
	

	
	return com_out;
}	


Vector3d CoMAdmittanceControl::ControlNoDelay(Vector2d com_d, Vector2d zmp_d, Vector2d zmp, double roll, double pitch, double omega_roll, double omega_pitch, bool GroundContact)
{
	Vector3d com_out;

	com_out = Vector3d(com_d(0),com_d(1),NaoRobot.getWalkParameter(ComZ));


	if(GroundContact)
	{
		double capture_pitch, capture_roll;

		capture_pitch  = -(1.0+K_pitch/NaoRobot.getWalkParameter(omega))*NaoRobot.getWalkParameter(ComZ) * (pitch + 1.0/NaoRobot.getWalkParameter(omega) * omega_pitch);
		capture_roll  =  (1.0+K_roll/NaoRobot.getWalkParameter(omega))*NaoRobot.getWalkParameter(ComZ) * (roll + 1.0/NaoRobot.getWalkParameter(omega) * omega_roll);
		
		zmp_out = zmp_d  + Vector2d(capture_pitch, capture_roll); 


		

		if(!firstrun)
		{
			Vector2d temp;
			k_f(0,0) = gain_x;
			k_f(1,1) = gain_y;
			//Force control for Trajectory Tracking
			if(k_f(0,0)>0)
			{	
				ZMPXint.add(zmp(0)-zmp_d(0),NaoRobot.getWalkParameter(Ts));
				int_zmp(0) = ZMPXint.eval();
				ZMPPXint.add(int_zmp(0),NaoRobot.getWalkParameter(Ts));
				int_zmpp(0) = ZMPPXint.eval();

			}

			if(k_f(1,1)>0)
			{
				ZMPYint.add(zmp(1)-zmp_d(1),NaoRobot.getWalkParameter(Ts));
				int_zmp(1) = ZMPYint.eval();
				ZMPPYint.add(int_zmp(1),NaoRobot.getWalkParameter(Ts));
				int_zmpp(1) = ZMPPYint.eval();
			}

			temp = com_d + k_f * int_zmpp;
			com_out = Vector3d(temp(0),temp(1),NaoRobot.getWalkParameter(ComZ));	
		}
		else
		{
			firstrun = false;
		}
	}

	
	return com_out;
}	

Vector3d CoMAdmittanceControl::Control(Vector2d com_d,Vector2d comd_d, Vector2d vrp_d, Vector2d vrp, Vector2d com)
{
    


    k_f(0,0) = gain_x;
    k_f(1,1) = gain_y;

	Vector2d temp;
   	if(firstrun)
	{
		firstrun = false;
		com_c = Vector3d(com_d(0),com_d(1),NaoRobot.getWalkParameter(ComZ));
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
	
		 temp = com_d + k_f * int_zmpp;
		 com_c = Vector3d(temp(0),temp(1),NaoRobot.getWalkParameter(ComZ));	
	
	}


    return com_c;
    
    
}

