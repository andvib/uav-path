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

    DifferentialState v, p, r, phi, psi, aileron, rudder;

    Control d_aileron;
    Control d_rudder;

    //IntermediateState p_N_dot, p_E_dot, p_D_dot;
    //IntermediateState Va, alpha, beta;
    //IntermediateState GAMMA, CHI;

    DifferentialEquation f;

    // Lateral state-space model coefficients
    double Yv, Yp, Yr, Yda, Ydr;
    double Lv, Lp, Lr, Lda, Ldr;
    double Nv, Np, Nr, Nda, Ndr;

    // Misc
    const double g = 9.81;
    const double PI = 3.14;

    //Va = sqrt(u*u + v*v + w*w);
    //alpha = atan(w/u);
    //beta = asin(v/Va);



    //_________________________________________________________________
    /* Trimmed State Variables */

    const double u_trim = 35.0;
    const double v_trim =  0.0;
    const double w_trim =  0.0;

    const double phi_trim   = 0.0;
    const double theta_trim = 0.066;
    const double psi_trim   = 0.0;

    const double p_trim = 0.0;
    const double q_trim = 0.0;
    const double r_trim = 0.0;

    const double elevator_trim = -0.05;
    const double aileron_trim  = 0.0;
    const double rudder_trim   = 0.0;
    const double throttle_trim = 0.0978;

    const double Va_trim    = 35.0;
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
    /* Control Differential Equation */

    f << dot(aileron)  == d_aileron;
    f << dot(rudder)   == d_rudder;
    


    //_________________________________________________________________
    /* Least Squares Function */

    Function cost;
    DMatrix Q(1,1); Q.setIdentity();
    DVector R(1);   R.setAll(0.0);

    cost << psi;



    //_________________________________________________________________
    /* Configure OCP */

    const double t_start = 0.0;
    const double t_end   = 10.0;
    const int    samples = 10*(t_end-t_start);

    OCP ocp( t_start, t_end, samples );

    ocp.minimizeLSQ( Q, cost, R );
    ocp.minimizeLSQEndTerm( Q, cost, R );
    ocp.subjectTo( f );



    //_________________________________________________________________
    /* Start Configuration */

    ocp.subjectTo( AT_START, v == 0 );
    
    ocp.subjectTo( AT_START, phi == 0.5 );
    ocp.subjectTo( AT_START, psi == 0 );

    ocp.subjectTo( AT_START, p == 0 );
    ocp.subjectTo( AT_START, r == 0 );

    ocp.subjectTo( AT_START, aileron == 0     );
    ocp.subjectTo( AT_START, rudder  == 0     );

    ocp.subjectTo( AT_START, d_aileron == 0.0 );
    ocp.subjectTo( AT_START, d_rudder  == 0.0 );



    //_________________________________________________________________
    /* Constraints */
    
    ocp.subjectTo( -PI/6 <= aileron  <= PI/6 );
    ocp.subjectTo( -PI/6 <= rudder   <= PI/6 );

    

    //_________________________________________________________________
    /* Configure Solver Algorithm */
    
    OptimizationAlgorithm algorithm(ocp);

    //algorithm.set( LEVENBERG_MARQUARDT, 100.0 );
    //algorithm.set( KKT_TOLERANCE, 10e-60 );
    //algorithm.set( MAX_NUM_ITERATIONS, 10);
    //algorithm.set( INTEGRATOR_TYPE, INT_RK78 );

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
