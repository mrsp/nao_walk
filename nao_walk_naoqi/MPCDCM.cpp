#include "MPCDCM.h"
MPCDCM::MPCDCM(RobotParameters &robot):NaoRobot(robot), dObsDCMx(robot), dObsDCMy(robot)
{
    

    //State xe is Delta dcm Delta vrp vrp
    Ae.resize(3,3);
    Be.resize(3);
    Ce.resize(3);
    Cx.resize(3);



    //Embedded Integrator DCM VRP
    A.resize(2,2);
    A.setZero();
    A(0,0) = NaoRobot.getWalkParameter(omega);
    A(0,1) = -NaoRobot.getWalkParameter(omega);
    A *= NaoRobot.getWalkParameter(Ts);
    A += Matrix2f::Identity();
    B.resize(2);
    B.setZero();
    B(1) = 1.000;
    B = B *NaoRobot.getWalkParameter(Ts);

    C.resize(2);
    C.setZero();
    C(1) = 1.000;


    Ae.setZero();
    Ae.block<2,2>(0,0) = A;
    Ae.block<1,2>(2,0) = C.transpose()* A;
    Ae(2,2) = 1.000;
    Be.setZero();
    Be(0) = B(0);
    Be(1) = B(1);
    Be(2) = C.transpose()*B;
    Ce.setZero();
    Ce(2) = 1.000;
    Cx.setZero();
    Cx(0) = 1.000;





	
    x.setZero();
    y.setZero();
    x_.setZero();
    y_.setZero();
    xe.setZero();
    ye.setZero();


    //VRP
    Fv.resize(Np,3);
    Fv.setZero();
    Fvu.resize(Np,Np);
    Fvu.setZero();

    //DCM
    Fx.resize(Np,3);
    Fx.setZero();
    Fxu.resize(Np,Np);
    Fxu.setZero();

    tmpb.resize(1,Np-1);
    temp.resize(3);
    Fv.block<1,3>(0,0)=Ce.transpose()*Ae;
    Fx.block<1,3>(0,0)=Cx.transpose()*Ae;
    Fv.block<1,3>(1,0)=Ce.transpose()*Ae*Ae;
    Fx.block<1,3>(1,0)=Cx.transpose()*Ae*Ae;
  

    temp = Be;
    Fvu(0,0) = Ce.transpose()*Be;
    Fxu(0,0) = Cx.transpose()*Be;
    Fvu(0,1) = Ce.transpose()*Ae*temp;
    Fxu(0,1) = Cx.transpose()*Ae*temp;

    Fvu(1,1) = Ce.transpose()*Be;
    Fxu(1,1) = Cx.transpose()*Be;



    for (unsigned int i = 2; i < Np; i++)
    {
  

        Fv.block<1,3>(i,0) =  Fv.block<1,3>(i-1,0) * Ae;
	    tmpb = Fvu.block<1,Np-1>(i-1,0);
        Fvu.block<1,Np-1>(i,1) = tmpb;
        temp = Ae*temp;
        Fvu(i,0) =  Ce.transpose()*temp;


        Fx.block<1,3>(i,0) =  Fx.block<1,3>(i-1,0) * Ae;
	    tmpb = Fxu.block<1,Np-1>(i-1,0);
        Fxu.block<1,Np-1>(i,1) = tmpb;
	    Fxu(i,0) =  Cx.transpose()*temp;
    }


    R.resize(Np,Np);
    R.setIdentity();
    R*=1.0e-4;

    qx = 0.05;
    qv = 0.02;

   
    Qv.resize(Np,Np);
    Qv.setIdentity();
    Qv = Qv*qv;

    Qx.resize(Np,Np);
    Qx.setIdentity();
    Qx = Qx*qx;

    //Hessian Matrix
    H.resize(Np,Np);
    H_inv.resize(Np,Np);
    H.setZero();
    H = R;
    H.noalias() += Fxu.transpose()*Qx*Fxu;
    H.noalias() += Fvu.transpose()*Qv*Fvu;

    //Make Symmetric
    H = (H+H.transpose())/2.0;
    //Compute the Gains
    H_inv = H.inverse();
    


    K_X.resize(Np,3);
    K_X = -H_inv*(Fxu.transpose()*Qx*Fx + Fvu.transpose() * Qv * Fv);
    K_x.resize(3);
    K_x(0) = K_X(0,0);
    K_x(1) = K_X(0,1);
    K_x(2) = K_X(0,2);

    K_V.resize(Np,Np);
    K_V = H_inv * Fvu.transpose()*Qv;
    K_v.resize(Np,1);
    K_v = K_V.block<1,Np>(0,0);


    cout<<"K_x "<<K_x<<endl;
    cout<<"K_v "<<K_v<<endl;

    VRPRefX.resize(Np,1);  
    VRPRefX.setZero();
    VRPRefY.resize(Np,1);  
    VRPRefY.setZero();
    DCM_.setZero();

    dcmx_d = 0;
    comx_d = 0;
    comdx_d = 0;
    dcmdx_d = 0;
    vrpx_d = 0;
    dcmy_d = 0;
    comy_d = 0;
    comdy_d = 0;
    dcmdy_d = 0;
    vrpy_d = 0;
    u_x = 0;
    u_y = 0;



    cout<<"MPC DCM Controller Initialized"<<endl;

}

void MPCDCM::setInitialState(Vector2f DCM, Vector2f CoM, Vector2f VRP)
{

    // dObsDCMx.setInitialState(Vector4f(CoM(0),DCM(0),VRP(0),0.0));
    // dObsDCMy.setInitialState(Vector4f(CoM(1),DCM(1),VRP(1),0.0));
   
   
    // xe(2) = VRP(0);
    // ye(2) = VRP(1);


    dObsDCMx.setInitialState(Vector4f(0,0,0,0));
    dObsDCMy.setInitialState(Vector4f(0,0,0,0));
   
   
    xe(2) = 0.0;
    ye(2) = 0.0;
}



void MPCDCM::Control(boost::circular_buffer<KVecFloat3> & VRPRef, Vector2f DCM, Vector2f CoM,  Vector2f VRP)
{

    


    for (unsigned int i = 0; i < Np; i++)
    {
        if (i+1 < VRPRef.size())
        {
            VRPRefX(i) = VRPRef[i+1](0);
            VRPRefY(i) = VRPRef[i+1](1);
        }
        else
        {
            VRPRefX(i) = VRPRef[VRPRef.size() - 1](0);
            VRPRefY(i) = VRPRef[VRPRef.size() - 1](1);

        }
     }


	

	xe(0) = x(1)-x_(1);
    xe(1) = x(2)-x_(2);
    xe(2) = x(2);
	
	ye(0) = y(1)-y_(1);
    ye(1) = y(2)-y_(2);
    ye(2) = y(2);

	//Optimal MPC Law
	du_x =  K_x.transpose()*xe;
    du_x += K_v.transpose()*VRPRefX;

    du_y =  K_x.transpose()*ye;
    du_y += K_v.transpose()*VRPRefY;

	//Desired VRP Velocity
	u_x += du_x;
	u_y += du_y;


	//Observer for MPC 
	x_ = x;
	y_ = y;  
    
    
	dObsDCMx.update(u_x,VRP(0),CoM(0));
    dObsDCMy.update(u_y,VRP(1),CoM(1));
    x =  dObsDCMx.getState();
    y =  dObsDCMy.getState();
    
    cout<<"State X "<<x<<endl;
    cout<<"State Y "<<y<<endl;
	//Desired Gait Pattern Reference
	comx_d = x(0);
	comy_d = y(0);
	dcmx_d = x(1);
	dcmy_d = y(1);
   	vrpx_d = x(2);
    vrpy_d = y(2);



	    dcmdx_d = NaoRobot.getWalkParameter(omega)*(dcmx_d - vrpx_d);
	    dcmdy_d = NaoRobot.getWalkParameter(omega)*(dcmy_d - vrpy_d);
	    comdx_d = -NaoRobot.getWalkParameter(omega)*(comx_d-dcmx_d);
	    comdy_d = -NaoRobot.getWalkParameter(omega)*(comy_d-dcmy_d);
	    comddx_d = NaoRobot.getWalkParameter(omega)*NaoRobot.getWalkParameter(omega)*(comx_d - vrpx_d);
	    comddy_d = NaoRobot.getWalkParameter(omega)*NaoRobot.getWalkParameter(omega)*(comy_d - vrpy_d);



   
}
