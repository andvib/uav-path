#include <acado_optimal_control.hpp>
#include "aerosonde_param.hpp"

#include <acado_gnuplot.hpp>

#include <iostream>
#include <fstream>


using namespace std;

int main(){

    USING_NAMESPACE_ACADO


    //_________________________________________________________________
    /* Introduce Variables */

    DifferentialState p_N, p_E, h;
    DifferentialState u, v, w;
    DifferentialState phi, theta, psi;
    DifferentialState p, q, r;
    DifferentialState elevator, aileron, rudder, throttle;

    Control d_elevator;
    Control d_aileron;
    Control d_rudder;
    Control d_throttle;

    IntermediateState p_N_dot, p_E_dot, p_D_dot;
    IntermediateState Va, alpha, beta;
    IntermediateState GAMMA, CHI;

    DifferentialEquation f;

    // Lateral state-space model coefficients
    double Yv, Yp, Yr, Yda, Ydr;
    double Lv, Lp, Lr, Lda, Ldr;
    double Nv, Np, Nr, Nda, Ndr;

    // Longitudinal state-space model coefficients
    double Xu, Xw, Xq, Xde, Xdt;
    double Zu, Zw, Zq, Zde;
    double Mu, Mw, Mq, Mde;
    double C_L, C_D;
    double C_X0, C_Xalp, C_Xde, C_Xq;
    double C_Z0, C_Zalp, C_Zde, C_Zq;

    // Misc
    const double g = 9.81;
    const double PI = 3.14;

    Va = sqrt(u*u + v*v + w*w);
    alpha = atan(w/u);
    beta = asin(v/Va);



    //_________________________________________________________________
    /* Trimmed State Variables */

    const double u_trim = 25.0;
    const double v_trim =  0.0;
    const double w_trim =  0.0;

    const double phi_trim   = 0.0;
    const double theta_trim = -0.12;
    const double psi_trim   = 0.0;

    const double p_trim = 0.0;
    const double q_trim = 0.0;
    const double r_trim = 0.0;

    const double elevator_trim = 0.0594;
    const double aileron_trim  = 0.0;
    const double rudder_trim   = 0.0;
    const double throttle_trim = 0.9;

    const double Va_trim    = 25.0;
    const double beta_trim  =  0.0;
    const double alpha_trim =  0.0;



    //_________________________________________________________________

    double gamma = J_x*J_z - J_xz*J_xz;
    double gamma_1 = J_xz*(J_x-J_y+J_z)/gamma;
    double gamma_2 = (J_z*(J_z-J_y)+J_xz*J_xz)/gamma;
    double gamma_3 = J_z/gamma;
    double gamma_4 = J_xz/gamma;
    double gamma_5 = (J_z - J_x)/J_y;
    double gamma_6 = J_xz/J_y;
    double gamma_7 = ((J_x-J_y)*J_x+J_xz*J_xz)/gamma;
    double gamma_8 = J_x/gamma;

    double C_p0 = gamma_3*C_l0    + gamma_4*C_n0;
    double C_pbeta = gamma_3*C_lbeta + gamma_4*C_nbeta;
    double C_pp = gamma_3*C_lp    + gamma_4*C_np;
    double C_pr = gamma_3*C_lr    + gamma_4*C_nr;
    double C_pda = gamma_3*C_lda   + gamma_4*C_nda;
    double C_pdr = gamma_3*C_ldr   + gamma_4*C_ndr;
    double C_r0 = gamma_4*C_l0    + gamma_8*C_n0;
    double C_rbeta = gamma_4*C_lbeta + gamma_8*C_nbeta;
    double C_rp = gamma_4*C_lp    + gamma_8*C_np;
    double C_rr = gamma_4*C_lr    + gamma_8*C_nr;
    double C_rda = gamma_4*C_lda   + gamma_8*C_nda;
    double C_rdr = gamma_4*C_ldr   + gamma_8*C_ndr;



    //_________________________________________________________________
    /* Force Coefficients: X and Z */

    C_L = C_L0 + C_Lalp*alpha_trim;
    C_D = C_D0 + C_Dalp*alpha_trim;

    C_X0 = 0;
    C_Xalp = -C_D*cos(alpha_trim) + C_L*sin(alpha_trim);
    C_Xde = -C_Dde*cos(alpha_trim) + C_Lde*sin(alpha_trim);
    C_Xq = -C_Dq*cos(alpha_trim) + C_Lq*sin(alpha_trim);

    C_Z0 = 0;
    C_Zalp = -C_D*sin(alpha_trim) - C_L*cos(alpha_trim);
    C_Zde = -C_Dde*sin(alpha_trim) - C_Lde*cos(alpha_trim);
    C_Zq = -C_Dq*sin(alpha_trim) - C_Lq*cos(alpha_trim);
    


    /*C_X0   = -C_D0;
    C_Xalp = -C_D0*(1-(alpha_trim*alpha_trim/2))  + C_L0*alpha_trim;
    C_Xde  = -C_Dde*(1-(alpha_trim*alpha_trim/2)) + C_Lde*alpha_trim;
    C_Xq   = -C_Dq*(1-(alpha_trim*alpha_trim/2))  + C_Lq*alpha_trim;


    C_Z0   = -C_L0;
    C_Zalp = -C_D0*alpha_trim  - C_L0*(1-(alpha_trim*alpha_trim/2));
    C_Zde  = -C_Dde*alpha_trim - C_Lde*(1-(alpha_trim*alpha_trim/2));
    C_Zq   = -C_Dq*alpha_trim  - C_Lq*(1-(alpha_trim*alpha_trim/2));*/



    //_________________________________________________________________
    /* Lateral State-Space model coefficients */

    Yv = ((rho*S*b*v_trim)/(4*mass*Va_trim))*(C_Yp*p_trim + C_Yr*r_trim) \
       + ((rho*S*v_trim)/mass)*(C_Y0 + C_Ybeta*beta_trim \
                                 + C_Yda*aileron_trim + C_Ydr*rudder_trim) \
       + ((rho*S*C_Ybeta)/(2*mass))*sqrt(u_trim*u_trim + w_trim*w_trim);

    Yp =  w_trim + ((rho*Va_trim*S*b)/(4*mass))*C_Yp;
    Yr = -u_trim + ((rho*Va_trim*S*b)/(4*mass))*C_Yr;

    Yda = ((rho*Va_trim*Va_trim*S)/(2*mass))*C_Yda;
    Ydr = ((rho*Va_trim*Va_trim*S)/(2*mass))*C_Ydr;
    
    Lv = ((rho*S*b*b*v_trim)/(4*Va_trim))*(C_pp*p_trim + C_pr*r_trim) \
       + rho*S*b*v_trim*(C_p0 + C_pbeta*beta_trim \
                            + C_pda*aileron_trim + C_pdr*rudder_trim) \
       + ((rho*S*b*C_pbeta)/2)*sqrt(u_trim*u_trim + w_trim*w_trim);      

    Lp = gamma_1*q_trim + ((rho*Va_trim*S*b*b)/4)*C_pp;
    Lr = -gamma_2*q_trim + ((rho*Va_trim*S*b*b)/4)*C_pr;

    Lda = ((rho*Va_trim*Va_trim*S*b)/2)*C_pda;
    Ldr = ((rho*Va_trim*Va_trim*S*b)/2)*C_pdr;

    Nv = ((rho*S*b*b*v_trim)/(4*Va_trim))*(C_rp*p_trim + C_rr*r_trim) \
       + rho*S*b*v_trim*(C_r0 + C_rbeta*beta_trim \
                            + C_rda*aileron_trim + C_rdr*rudder_trim) \
       + ((rho*S*b*C_rbeta)/2)*sqrt(u_trim*u_trim + w_trim*w_trim);

    Np =  gamma_7*q_trim + ((rho*Va_trim*S*b*b)/4)*C_rp;
    Nr = -gamma_1*q_trim + ((rho*Va_trim*S*b*b)/4)*C_rr;

    Nda = ((rho*Va_trim*Va_trim*S*b)/2)*C_rda;
    Ndr = ((rho*Va_trim*Va_trim*S*b)/2)*C_rdr;



    //_________________________________________________________________
    /* Longitudinal State-Space Model Coefficients */

    Xu = ((u_trim*rho*S)/mass)*(C_X0 + C_Xalp*alpha_trim + C_Xde*elevator_trim)\
       - ((rho*S*w_trim*C_Xalp)/(2*mass)) \
       + ((rho*S*c*C_Xq*u_trim*q_trim)/(4*mass*Va_trim)) \
       - ((rho*S_prop*C_prop*u_trim)/mass);

    Xw = -q_trim+((w_trim*rho*S)/mass)*(C_X0 + C_Xalp*alpha_trim + C_Xde*elevator_trim)\
       + ((rho*S*c*C_Xq*w_trim*q_trim)/(4*mass*Va_trim)) \
       + ((rho*S*C_Xalp*u_trim)/(2*mass)) - ((rho*S_prop*C_prop*w_trim)/mass);

    Xq  = -w_trim + ((rho*Va_trim*S*C_Xq*c)/(4*mass));
    Xde = ((rho*Va_trim*Va_trim*S*C_Xde)/(2*mass));
    Xdt = ((rho*S_prop*C_prop*k_motor*k_motor*throttle_trim)/mass);

    
    Zu = q_trim+ ((u_trim*rho*S)/mass)*(C_Z0 + C_Zalp*alpha_trim + C_Zde*elevator_trim)\
       - ((rho*S*C_Zalp*w_trim)/(2*mass)) \
       + ((u_trim*rho*S*C_Zq*c*q_trim)/(4*mass*Va_trim));
    
    Zw = ((w_trim*rho*S)/mass)*(C_Z0 + C_Zalp*alpha_trim + C_Zde*elevator_trim)\
       + ((rho*S*C_Zalp*u_trim)/(2*mass)) \
       + ((rho*w_trim*S*c*C_Zq*q_trim)/(4*mass*Va_trim));

    Zq = u_trim + ((rho*Va_trim*S*C_Zq*c)/(4*mass));
    Zde = ((rho*Va_trim*Va_trim*S*C_Zde)/(2*mass));


    Mu = ((u_trim*rho*S*c)/J_y)*(C_m0 + C_malp*alpha_trim + C_mde*elevator_trim)\
       - ((rho*S*c*C_malp*w_trim)/(2*J_y)) \
       + ((rho*S*c*c*C_mq*q_trim*u_trim)/(4*J_y*Va_trim));

    Mw = ((w_trim*rho*S*c)/J_y)*(C_m0 + C_malp*alpha_trim + C_mde*elevator_trim) \
       + ((rho*S*c*C_malp*u_trim)/(2*J_y)) \
       + ((rho*S*c*c*C_mq*q_trim*w_trim)/(4*J_y*Va_trim));

    Mq = ((rho*Va_trim*S*c*c*C_mq)/(4*J_y));
    Mde = ((rho*Va_trim*Va_trim*S*c*C_mde)/(2*J_y));



    //_________________________________________________________________
    /* Position Differential Equations */

    p_N_dot = cos(theta)*cos(psi)*u \
            + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v \
            + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
	
    p_E_dot = cos(theta)*sin(psi)*u \
            + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v \
            + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;

    p_D_dot = -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;


    f << dot(p_N) == p_N_dot;
    f << dot(p_E) == p_E_dot;


    //_________________________________________________________________
    /* Lateral Differential Equations */

    f << dot(v) == Yv*v + Yp*p + Yr*r + g*cos(theta_trim)*cos(phi_trim)*phi \
                 + Yda*aileron + Ydr*rudder;
    
    f << dot(p) == Lv*v + Lp*p + Lr*r + Lda*aileron + Ldr*rudder;

    f << dot(r) == Nv*v + Np*p + Nr*r + Nda*aileron + Ndr*rudder;

    f << dot(phi) == p + cos(phi_trim)*tan(theta_trim)*r   \
                   + (q_trim*cos(phi_trim)*tan(theta_trim) \
                   - r_trim*sin(phi_trim)*tan(theta_trim))*phi;

    f << dot(psi) == cos(phi_trim)*(1/cos(theta_trim))*r \
                   + (p_trim*cos(phi_trim)*(1/cos(theta_trim)) \
                   -  r_trim*sin(phi_trim)*(1/cos(theta_trim)))*phi;



    //_________________________________________________________________
    /* Longitudinal Differential Equations */

    f << dot(u) == Xu*u + Xw*w + Xq*q - g*cos(theta_trim)*theta \
                                        + Xde*elevator + Xdt*throttle;

    f << dot(w) == Zu*u + Zw*w + Zq*q - g*sin(theta_trim)*theta + Zde*elevator;

    f << dot(q) == Mu*u + Mw*w + Mq*q + Mde*elevator;

    f << dot(theta) == q;

    f << dot(h) == sin(theta_trim)*u - cos(theta_trim)*w \
                    + (u_trim*cos(theta_trim)+w_trim*sin(theta_trim))*theta;



    //_________________________________________________________________
    /* Control Differential Equation */

    f << dot(elevator) == d_elevator;
    f << dot(aileron)  == d_aileron;
    f << dot(rudder)   == d_rudder;
    f << dot(throttle) == d_throttle;



    //_________________________________________________________________
    /* Calculate Gamma and Chi */
    
    GAMMA = -atan(p_D_dot/sqrt(p_N_dot*p_N_dot + p_E_dot*p_E_dot + p_D_dot*p_D_dot)); 
    CHI = asin(p_E_dot/sqrt(p_N_dot*p_N_dot + p_E_dot*p_E_dot + p_D_dot*p_D_dot));



    //_________________________________________________________________
    /* Least Squares Function */

    Function cost; const int no = 7;
    DMatrix Q(no,no); Q.setIdentity();
    DVector R(no);   R.setAll(0.0);

    cost << d_elevator;
    cost << d_aileron;
    cost << d_rudder;
    cost << d_throttle;

    cost << u; R(4) = 25;  Q(4,4) = 1;
    cost << h; R(5) = 150; Q(5,5) = 1;
    //cost << psi; R(6) = 0.0;



    //_________________________________________________________________
    /* Path */

    VariablesGrid path;
    path.read( "./../path.txt" );
    cout << "Path to follow: \n";
    path.print();

    cout << "\n";

    Function trajectory;

    trajectory << p_N;
    trajectory << p_E;
    trajectory << h;
    trajectory << u;

    DMatrix Q2(4,4); Q2.setIdentity();

    Q(2,2) = 100;



    Function end;

    end << p_N;
    end << p_E;
    end << h;
    end << u;

    DMatrix QEnd(4,4); QEnd.setIdentity();
    DVector REnd(4);
    REnd(0) = 187.5;
    REnd(1) = 62.5;
    REnd(2) = 150.0;
    REnd(3) = 25.0;

    //_________________________________________________________________
    /* Configure OCP */

    const double t_start = 0.0;
    const double t_end   = 10.0;
    const int    samples = 10*(t_end-t_start);

    OCP ocp( path.getTimePoints() );//t_start, t_end, samples );
    
    //ocp.minimizeLSQ( Q, cost, R );
    //ocp.minimizeLSQEndTerm( QEnd, end, REnd );

    ocp.minimizeLSQ( Q2, trajectory, path );

    ocp.subjectTo( f );



    //_________________________________________________________________
    /* Start Configuration */
    
    ocp.subjectTo( AT_START, p_N == 0   );
    ocp.subjectTo( AT_START, p_E == 0   );
    ocp.subjectTo( AT_START,   h == 150 );

    ocp.subjectTo( AT_START, u == 25.0 );
    ocp.subjectTo( AT_START, v == 0.0 );
    ocp.subjectTo( AT_START, w == 0.0 );
    
    ocp.subjectTo( AT_START, phi   == 0.0   );
    ocp.subjectTo( AT_START, theta == 0.066 );
    ocp.subjectTo( AT_START, psi   == 0.0   );

    ocp.subjectTo( AT_START, p == 0 );
    ocp.subjectTo( AT_START, q == 0 );
    ocp.subjectTo( AT_START, r == 0 );

    /*ocp.subjectTo( AT_START, elevator == 0.0594 );
    ocp.subjectTo( AT_START,  aileron == 0.0 );
    ocp.subjectTo( AT_START,   rudder == 0.0 );
    ocp.subjectTo( AT_START, throttle == 0.0978  );*/

    ocp.subjectTo( AT_START, d_elevator == 0.0 );
    ocp.subjectTo( AT_START, d_aileron  == 0.0 );
    ocp.subjectTo( AT_START, d_rudder   == 0.0 );
    ocp.subjectTo( AT_START, d_throttle == 0.0 );



    //_________________________________________________________________
    /* Constraints */

    ocp.subjectTo( -PI/2 <= theta <= PI/2 );
    ocp.subjectTo( -PI/2 <= phi <= PI/2 );

    ocp.subjectTo( -PI/6 <= elevator <= PI/6 );
    ocp.subjectTo( -PI/6 <= aileron  <= PI/6 );
    ocp.subjectTo( -PI/6 <= rudder   <= PI/6 );
    ocp.subjectTo(     0 <= throttle <= 1 );

    ocp.subjectTo( -0.5 <= d_throttle <= 0.5 );
    //ocp.subjectTo( -1 <= d_elevator <= 1 );


    //_________________________________________________________________
    /* Configure Solver Algorithm */
    
    OptimizationAlgorithm algorithm(ocp);

    //algorithm.set( LEVENBERG_MARQUARDT, 100.0 );
    //algorithm.set( KKT_TOLERANCE, 10e-10 );
    //algorithm.set( MAX_NUM_ITERATIONS, 100);
    algorithm.set( INTEGRATOR_TYPE, INT_RK78 );

    //_________________________________________________________________
    /* Prepare Solutions */

    LogRecord logRecord( LOG_AT_END );
    logRecord.setPrintScheme( PS_MATLAB );
    logRecord.addItem( LOG_DIFFERENTIAL_STATES, "STATES" );
    logRecord.addItem( LOG_CONTROLS, "CONTROLS" );   

    algorithm << logRecord;
    


    //_________________________________________________________________

    algorithm.solve();



    //_________________________________________________________________
    /* Results */

    algorithm.getLogRecord( logRecord );

    ofstream myfile;
    myfile.open( "./../results/output.m" );
    logRecord.print( myfile );
    myfile.close();


    return 0;
}
