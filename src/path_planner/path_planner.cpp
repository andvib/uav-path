#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include "aerosonde_param.hpp"

#define PI 3.14
#define G  9.81

int main(){

    USING_NAMESPACE_ACADO

    //_________________________________________________________________
    /* Introduce Variables */
    DifferentialState p_N, p_E, p_D;
    DifferentialState u, v, w;
    DifferentialState q0, q1, q2, q3;
    DifferentialState p, q, r;
    DifferentialState elevator, aileron, rudder, throttle;

    Control d_elevator;
    Control d_aileron;
    Control d_rudder;
    Control d_throttle;

    IntermediateState f_x, f_gx, f_ax, f_px, Cx, Cxq, Cxde;
    IntermediateState f_y, f_gy, f_ay;
    IntermediateState f_z, f_gz, f_az, Cz, Czq, Czde;
    IntermediateState l, m, n;
    IntermediateState Va, alpha, beta;
    IntermediateState aero, C_D, C_L;
    IntermediateState p_N_dot, p_E_dot, p_D_dot;
    IntermediateState CHI, GAMMA;
    IntermediateState e0, e1, e2, e3, quatLength;

    DifferentialEquation f;


    //_________________________________________________________________
    /* Airdata */
    Va = sqrt(u*u + v*v + w*w);
    alpha = atan(w/u);
    beta = asin(v/Va);

    aero = 0.5*rho*Va*Va*S;

    quatLength = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    
    e0 = q0/quatLength;
    e1 = q1/quatLength;
    e2 = q2/quatLength;
    e3 = q3/quatLength;


    //_________________________________________________________________
    /* Forces */
    C_L = C_L0 + C_Lalp*alpha;
    C_D = C_D0 + C_Dalp*alpha;

    // Forces in x-direction
    f_gx = 2*G*mass * (e1*e3 - e2*e0);

    Cx   = -C_D*cos(alpha) + C_L*sin(alpha);
    Cxq  = -C_Dq*cos(alpha) + C_Lq*sin(alpha);
    Cxde = -C_Dde*cos(alpha) + C_Lde*sin(alpha);
    f_ax = aero * (Cx + Cxq*(c/(2.*Va))*q + Cxde*elevator);

    f_px = 0.5*rho*S_prop*C_prop*(k_motor*k_motor*throttle*throttle - Va*Va);

    f_x = f_gx + f_ax + f_px;


    // Forces in y-direction
    f_gy = 2*G*mass*(e2*e3 - e1*e0);

    f_ay = aero*(C_Y0 + C_Ybeta*beta + C_Yp*(b*p/(2.*Va)) + C_Yr*(b*r/(2.*Va)) +\
           C_Yda*aileron + C_Ydr*rudder);

    f_y = f_gy + f_ay;


    // Forces in z-direction
    f_gz = G*mass * (e3*e3 + e0*e0 - e1*e1 - e2*e2);

    Cz   = -C_D*sin(alpha) - C_L*cos(alpha);
    Czq  = -C_Dq*sin(alpha) - C_Lq*cos(alpha);
    Czde = -C_Dde*sin(alpha) - C_Lde*cos(alpha);   
    f_az = aero * (Cz + Czq*(c*q/(2.*Va)) + Czde*elevator);

    f_z = f_gz + f_az;


    //_________________________________________________________________
    /* Moments */

    l = aero*b*(C_l0 + C_lbeta*beta + C_lbeta*beta + C_lp*(b/(2.*Va))*p +\
        C_lr*(b/(2.*Va))*r + C_lda*aileron + C_ldr*rudder) -\
        k_Tp*(k_omega*k_omega*throttle*throttle);

    m = aero*c*(C_m0 + C_malp*alpha + C_mq*(c/(2.*Va))*q + C_mde*elevator);

    n = aero*b*(C_n0 + C_nbeta*beta + C_np*(b/(2.*Va))*p + C_nr*(b/(2.*Va))*r +\
        C_nda*aileron + C_ndr*rudder);
    

    //_________________________________________________________________
    /* Differential Equation: Position */
    p_N_dot = (e1*e1 + e0*e0 - e2*e2 - e3*e3)*u +\
              2*(e1*e2 - e3*e0)*v + 2*(e1*e3 + e2*e0)*w;

    p_E_dot = 2*(e1*e2 + e3*e0)*u + (e2*e2 + e0*e0 - e1*e1 - e3*e3)*v +\
              2*(e2*e3 - e1*e0)*w;

    p_D_dot = -2*(e1*e3 - e2*e0)*u - 2*(e2*e3 + e1*e0)*v -\
              (e3*e3 + e0*e0 - e1*e1 - e2*e2)*w;

    f << dot(p_N) == p_N_dot;
    f << dot(p_E) == p_E_dot;
    f << dot(p_D) == p_D_dot;

    GAMMA = -atan(p_D_dot/sqrt(p_N_dot*p_N_dot + p_E_dot*p_E_dot + p_D_dot*p_D_dot));
    CHI   =  asin(p_E_dot/sqrt(p_N_dot*p_N_dot + p_E_dot*p_E_dot + p_D_dot*p_D_dot));


    //_________________________________________________________________
    /* Differential Equation: Velocity */
    
    f << dot(u) == r*v - q*w + (1/mass)*f_x;

    f << dot(v) == p*w - r*u + (1/mass)*f_y;

    f << dot(w) == q*u - p*v + (1/mass)*f_z;


    //_________________________________________________________________
    /* Differential Equation: Quaternions */

    f << dot(q0) == -0.5*(p*e1 + q*e2 + r*e3);
    
    f << dot(q1) ==  0.5*(p*e0 + r*e2 - q*e3);
    
    f << dot(q2) ==  0.5*(q*e0 - r*e1 + p*e3);
    
    f << dot(q3) ==  0.5*(r*e0 + q*e1 - p*e2);


    //_________________________________________________________________
    /* Differential Equation: Attitude Rates */

    f << dot(p) == gamma_1*p*q - gamma_2*q*r +\
                   0.5*rho*Va*Va*S*b*(C_p0 + C_pbeta*beta + C_pp*(b*p/(2.*Va)) +\
                   C_pr*(b*r/(2.*Va)) + C_pda*aileron + C_pdr*rudder);

    f << dot(q) == gamma_5*p*r - gamma_6*(p*p - r*r) +\
                   0.5*(rho*Va*Va*S*c/J_y)*(C_m0 + C_malp*alpha +\
                   C_mq*(c*q/(2.*Va)) + C_mde*elevator);

    f << dot(r) == gamma_7*p*q - gamma_1*q*r +\
                   0.5*rho*Va*Va*S*b*(C_r0 + C_rbeta*beta + C_rp*(b*p/(2.*Va)) +\
                   C_rr*(b*r/(2.*Va)) + C_rda*aileron + C_rdr*rudder);


    //_________________________________________________________________
    /* Differential Equation: Control */
    
    f << dot(elevator) == d_elevator;
    f << dot(aileron)  == d_aileron;
    f << dot(rudder)   == d_rudder;
    f << dot(throttle) == d_throttle;


    //_________________________________________________________________
    /* Define LSQ function */
    Function h;
    DMatrix weights(2,2); weights.setIdentity();
    DVector ref(2);       ref.setAll(0.0);

    h << GAMMA;
    h << CHI;


    //_________________________________________________________________
    /* Configure OCP */
    
    const double t_start = 0.0;
    const double t_end   = 10.0;
    const int    samples = 10*(t_end - t_start);

    
    OCP ocp( t_start, t_end, samples);

    ocp.minimizeLSQ( weights, h, ref);
    ocp.minimizeLSQEndTerm( weights, h, ref);
    ocp.subjectTo( f );


    //_________________________________________________________________
    /* Start Configuration */
    ocp.subjectTo( AT_START, p_N == 0   );
    ocp.subjectTo( AT_START, p_E == 0   );
    ocp.subjectTo( AT_START, p_D == 150 );

    ocp.subjectTo( AT_START, u == 35 );
    ocp.subjectTo( AT_START, v == 0  );
    ocp.subjectTo( AT_START, w == 0  );

    ocp.subjectTo( AT_START, q0 == 1.0 );
    ocp.subjectTo( AT_START, q1 == 0.0 );
    ocp.subjectTo( AT_START, q2 == 0.0 );
    ocp.subjectTo( AT_START, q3 == 0.0 );

    ocp.subjectTo( AT_START, p == 0.0 );
    ocp.subjectTo( AT_START, q == 0.0 );
    ocp.subjectTo( AT_START, r == 0.0 );

    ocp.subjectTo( AT_START, elevator == -0.08 );
    ocp.subjectTo( AT_START, aileron  == 0.0   );
    ocp.subjectTo( AT_START, rudder   == 0.0   );
    ocp.subjectTo( AT_START, throttle == 0.33  );

    ocp.subjectTo( AT_START, d_elevator == 0.0 );
    ocp.subjectTo( AT_START, d_aileron  == 0.0 );
    ocp.subjectTo( AT_START, d_rudder   == 0.0 );
    ocp.subjectTo( AT_START, d_throttle == 0.0 );


    //_________________________________________________________________
    /* Constraints */
    ocp.subjectTo( -PI/6 <= elevator <= PI/6 );
    ocp.subjectTo( -PI/6 <= aileron  <= PI/6 );
    ocp.subjectTo( -PI/6 <= rudder   <= PI/6 );
    ocp.subjectTo( -PI/6 <= throttle <= PI/6 );


    //_________________________________________________________________
    /* Configure solver algorithm */
    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( INTEGRATOR_TYPE, INT_RK78 );
    algorithm.set( DISCRETIZATION_TYPE, SINGLE_SHOOTING );
    
    algorithm.set( MAX_NUM_ITERATIONS, 10 );

    algorithm.set( ABSOLUTE_TOLERANCE, 100);
    algorithm.set( INTEGRATOR_TOLERANCE, 100);


    //_________________________________________________________________
    /* Prepare solution */
    GnuplotWindow windowStates;
    windowStates.addSubplot(u, "u");
    windowStates.addSubplot(p_D, "DOWN");
    windowStates.addSubplot(elevator, "ELEVATOR");
    windowStates.addSubplot(aileron, "AILERON");
    windowStates.addSubplot(rudder, "RUDDER");
    windowStates.addSubplot(throttle, "THROTTLE");
    windowStates.addSubplot(GAMMA, "GAMMA");
    windowStates.addSubplot(CHI, "CHI");



    algorithm << windowStates;



    algorithm.solve();



    return 0;

}
