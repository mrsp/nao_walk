#include "StepAdjustment.h"
StepAdjustment::StepAdjustment(RobotParameters &robot_):robot(robot_)
{
    dt = (double) robot.getWalkParameter(Ts);
    w = (double) robot.getWalkParameter(omega);
    T_max = (double) robot.getWalkParameter(Tss_max);
    T_min = (double) robot.getWalkParameter(Tss_min);
    T_nom = (double) robot.getWalkParameter(Tss);
    b_nom.zero();
    L_nom.zero();
    tau_min = exp(w*T_min);
    tau_max = exp(w*T_max);
    tau_nom = exp(w*T_nom);

    rot.zero();
    L_max.zero();
    L_min.zero();
    lp = (double) 2.0*robot.getWalkParameter(H0);
 
    //Set weights
    a_0 = 1.0;
    a_1 = 1.0;
    a_2 = 0.1;
    a_3 = 100.0;
    a_4 = 100.0;

    n = 5;
    G.resize(n, n);
    G[0][0] = 2.0*a_0;
    G[0][1] = 0.0;
    G[0][2] = 0.0;
    G[0][3] = 0.0;
    G[0][4] = 0.0;


    G[1][0] = 0.0;
    G[1][1] = 2.0*a_1;
    G[1][2] = 0.0;
    G[1][3] = 0.0;
    G[1][4] = 0.0;


    G[2][0] = 0.0;
    G[2][1] = 0.0;
    G[2][2] = 2.0*a_2;
    G[2][3] = 0.0;
    G[2][4] = 0.0;

    
    G[3][0] = 0.0;
    G[3][1] = 0.0;
    G[3][2] = 0.0;
    G[3][3] = 2.0*a_3;
    G[3][4] = 0.0;


    G[4][0] = 0.0;
    G[4][1] = 0.0;
    G[4][2] = 0.0;
    G[4][3] = 0.0;
    G[4][4] = 2.0*a_4;
    
 
    g0.resize(n);
    g0[0] = 0.0;
    g0[1] = 0.0;
    g0[2] = -2.0*a_2*tau_nom;
    g0[3] = 0.0;
    g0[4] = 0.0;

    m = 2;
    ce0.resize(m);
    ce0[0] = 0.0;
    ce0[1] = 0.0;
    CE.resize(n, m);
    CE[0][0]= 1.0;
    CE[1][0]= 0.0;
    CE[2][0]= 0.0;
    CE[3][0]= 1.0;
    CE[4][0]= 0.0;

    CE[0][1]= 0.0;
    CE[1][1]= 1.0;
    CE[2][1]= 0.0;
    CE[3][1]= 0.0;
    CE[4][1]= 1.0;

    p = 6;
    CI.resize(n, p);
    CI[0][0] = -1.0;
    CI[0][1] = 1.0;
    CI[0][2] = 0.0;
    CI[0][3] = 0.0;
    CI[0][4] = 0.0;
    CI[0][5] = 0.0;

    CI[1][0] = 0.0;
    CI[1][1] = 0.0;
    CI[1][2] = -1.0;
    CI[1][3] = 1.0;
    CI[1][4] = 0.0;
    CI[1][5] = 0.0;

   
    CI[2][0] = 0.0;
    CI[2][1] = 0.0;
    CI[2][2] = 0.0;
    CI[2][3] = 0.0;
    CI[2][4] = -1.0;
    CI[2][5] = 1.0;

    
    CI[3][0] = 0.0;
    CI[3][1] = 0.0;
    CI[3][2] = 0.0;
    CI[3][3] = 0.0;
    CI[3][4] = 0.0;
    CI[3][5] = 0.0;

    CI[4][0] = 0.0;
    CI[4][1] = 0.0;
    CI[4][2] = 0.0;
    CI[4][3] = 0.0;
    CI[4][4] = 0.0;
    CI[4][5] = 0.0;
   
    ci0.resize(p);
    ci0[4]= tau_max;
    ci0[5]= -tau_min;

    x.resize(n);
    cout<<"StepAdjustment in "<<axis<<" initialized successfully"<<endl;
  
}

void StepAdjustment::solve(double ksix_0,  double ksiy_0, double copx_0, double copy_0, double vrpx_0, double vrpy_0, double vrpx_ref, double vrpy_ref, double support_orientation, int RSS)
{

     rot.zero();
     KMath::KMat::transformations::makeRotation(rot,support_orientation);
     rot_T = rot.transp();
     L_nom(0) = vrpx_ref-vrpx_0;
     L_nom(1) = vrpy_ref-vrpy_0;
     L_nom = rot.transp() * L_nom;


 
        if(RSS==1)
        {
          Ly_max = (double) robot.getWalkParameter(MaxStepY);
          Ly_min = (double) robot.getWalkParameter(MinStepY);
          by_nom = -lp/(1+tau_nom)-(L_nom(1) -lp)/(1.0-tau_nom);
         
        }
        else
        {
          Ly_min = - (double) robot.getWalkParameter(MaxStepY);
          Ly_max = - (double) robot.getWalkParameter(MinStepY);
          by_nom = lp/(1+tau_nom)-(L_nom(1)+lp)/(1.0-tau_nom);
        }
  
        Lx_max = (double) robot.getWalkParameter(MaxStepX);
        Lx_min = (double) robot.getWalkParameter(MinStepX);
        bx_nom = L_nom(0)/(tau_nom-1.0);

    // //Rotate the Constraints to the support foot
    // L_max = rot * KVecDouble2(Lx_max,Ly_max);
    // L_min = rot * KVecDouble2(Lx_min,Ly_min);

    // Lx_max = L_max(0);
    // Ly_max = L_max(1);

    // Lx_min = L_min(0);
    // Ly_min = L_min(1);


    b_nom = rot * KVecDouble2(bx_nom,by_nom);
    bx_nom = b_nom(0);
    by_nom = b_nom(1);

    g0[0] = -2.0*a_0*(vrpx_ref);
    g0[1] = -2.0*a_1*(vrpy_ref);
    g0[3] = -2.0*a_3*bx_nom;
    g0[4] = -2.0*a_4*by_nom;


    //Equality constraints
    CE[2][0] = -(ksix_0-copx_0);
    CE[2][1] = -(ksiy_0-copy_0);

    ce0[0] = -copx_0;
    ce0[1] = -copy_0;


    //Inequality Constraints
    a_11 = rot_T(0,0);
    a_12 = rot_T(0,1);
    a_21 = rot_T(1,0);
    a_22 = rot_T(1,1);

    // a_11 = 1.0;
    // a_12 = 0.0;
    // a_21 = 0.0;
    // a_22 = 1.0;
    CI[0][0] = -a_11;
    CI[0][1] = a_11;
    CI[0][2] = -a_21;
    CI[0][3] = a_21;
    CI[0][4] = 0.0;
    CI[0][5] = 0.0;

    CI[1][0] = -a_12;
    CI[1][1] = a_12;
    CI[1][2] = -a_22;
    CI[1][3] = a_22;
    CI[1][4] = 0.0;
    CI[1][5] = 0.0;



    ci0[0] =  a_11 * vrpx_0 + a_12*vrpy_0 +  Lx_max;
    ci0[1] =  -a_11 * vrpx_0 - a_12*vrpy_0 - Lx_min;
    ci0[2] =  a_21 * vrpx_0 + a_22*vrpy_0 + Ly_max;
    ci0[3] =  -a_21 * vrpx_0 - a_22*vrpy_0 - Ly_min;


    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
    //std::cout << "f: " << f << std::endl;
    if(f>std::numeric_limits<double>::max()) //e.g. problem is infeasible, follow the nomimal gait pattern
    {
        step_locationx = vrpx_ref;
        step_locationy = vrpy_ref;
        step_duration = T_nom;
        step_instructions = ceil(step_duration/dt);
        step_bx = 0.0;
        step_by = 0.0;
    }
    else
    {
     
      step_locationx = x[0];
      step_locationy = x[1];
      step_duration = 1.0/w * log(x[2]);
      step_instructions = ceil(step_duration/dt);
      step_bx = x[3];
      step_by = x[4];
    }
      cout<<"Ref Location"<<endl;
      cout<<vrpx_ref<<" "<<vrpy_ref<<endl;
      cout<<"Initial Location"<<endl;
      cout<<vrpx_0<<" "<<vrpy_0<<endl;
      cout<<"Step Location"<<endl;
      cout<<step_locationx<<" "<<step_locationy<<endl;
      cout<<"Step Instructions"<<endl;
      cout<<step_instructions<<endl;
      cout<<"Step bx "<<step_bx<<"Step bx nom "<<bx_nom<<endl;
      cout<<"Step by "<<step_by<<"Step by nom "<<by_nom<<endl;
      cout<<"DCM "<<ksix_0<<" "<<ksiy_0<<endl;
}