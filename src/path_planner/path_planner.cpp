#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include "x8_param.hpp"

#define PI  3.14
#define  g  9.81



int main(){
    USING_NAMESPACE_ACADO

    /* UAV STATES */
    DifferentialState p_N, p_E, p_D;
    DifferentialState q0, q1, q2, q3;
    DifferentialState u, v, w;
    DifferentialState p, q, r;

    /* CONTROL STATES */
    Control elevator;
    Control aileron;
    Control throttle;

    /* INTERMEDIATE STATES */
    IntermediateState Cx, Cxq, Cxde;
    IntermediateState Cz, Czq, Czde;
	IntermediateState l, m, n;
	IntermediateState Va, alpha, beta;
    IntermediateState aero, C_D, C_L;
    IntermediateState quatLength;    
    IntermediateState e0, e1, e2, e3;

    /* DEFINE DIFFERENTIAL EQUATION */
    DifferentialEquation f;

    const double t_start =  0.0;
    const double t_end   =  10.0;


    //_________________________________________________________________
    /* AIRDATA */
    // ONLY VALID AS LONG AS THERE IS NO WIND
    Va = sqrt(u*u + v*v + w*w);
    alpha = atan(w/u);
    beta = asin(v/Va);    

    aero = 0.5*rho*Va*Va*S/mass;
    
    C_L = C_L0 + C_Lalp*alpha;
    C_D = C_D0 + C_Dalp*alpha;

    Cx   = -C_D*cos(alpha) + C_L*sin(alpha);
    Cxq  = -C_Dq*cos(alpha) + C_Lq*sin(alpha);
    Cxde = -C_Dde*cos(alpha) + C_Lde*sin(alpha);

    Cz   = -C_D*sin(alpha) - C_L*cos(alpha);
    Czq  = -C_Dq*sin(alpha) - C_Lq*cos(alpha);
    Czde = -C_Dde*sin(alpha) - C_Lde*cos(alpha);

    quatLength = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    
    e0 = q0/quatLength;
    e1 = q1/quatLength;
    e2 = q2/quatLength;
    e3 = q3/quatLength;


    //_________________________________________________________________
    /* DYNAMICS */
    
    // Position
    f << dot(p_N) == (e1*e1 + e0*e0 - e2*e2 - e3*e3)*u + \
                     2*(e1*e2 - e3*e0)*v + 2*(e1*e3 + e2*e0)*w;
    
    f << dot(p_E) == 2*(e1*e2 + e3*e0)*u + \
                     (e2*e2 + e0*e0 - e1*e1 - e3*e3)*v +\
                     2*(e2*e3 - e1*e0)*w;

    f << dot(p_D) == -2*(e1*e3 - e2*e0)*u - 2*(e2*e3 + e1*e0)*v - \
                     (e3*e3 + e0*e0 - e1*e1 - e2*e2)*w;


    // Velocity
    f << dot(u) == r*v - q*w + 2*g*(e1*e3 - e2*e0) + \
                   aero*(Cx + Cxq*(c*q/(2*Va)) + Cxde*elevator +\
              ((rho*S_prop*C_prop)/(2*mass))*(k_motor*k_motor*throttle*throttle-Va*Va));

    f << dot(v) == p*w - r*u + 2*g*(e2*e3 + e1*e0) + aero*\
                   (C_Y0 + C_Ybeta*beta + C_Yp*(b*p/(2*Va)) + C_Yr*(b*r/(2*Va)) + \
                    C_Yda*aileron);

    f << dot(w) == q*u - p*v + g*(e3*e3 + e0*e0 - e1*e1 - e2*e2) + \
                   aero*(Cz + Czq*(c*q/(2*Va)) + Czde*elevator);

    
    // Quaternion
    f << dot(q0) == -0.5*(p*e1 + q*e2 + r*e3);

    f << dot(q1) == 0.5*(p*e0 + r*e2 - q*e3);

    f << dot(q2) == 0.5*(q*e0 - r*e1 + p*e3);

    f << dot(q3) == 0.5*(r*e0 + q*e1 - p*e2);


    // Angle rates
    f << dot(p) == gamma_1*p*q - gamma_2*q*r + \
                   0.5*rho*Va*Va*S*b*(C_p0 + C_pbeta*beta + C_pp*(b*p/(2*Va)) +\
                   C_pr*(b*r/(2*Va)) + C_pda*aileron);

    f << dot(q) == gamma_5*p*r - gamma_6*(p*p - r*r) + ((rho*Va*Va*S*c)/(2*J_y))*\
                   (C_m0 + C_malp*alpha + C_mq*(c*q/(2*Va)) + C_mde*elevator);

    f << dot(r) == gamma_7*p*q - gamma_1*q*r + 0.5*rho*Va*Va*S*b*(C_r0 + \
                   C_rbeta*beta + C_rp*(b*p/(2*Va)) + C_rr*(b*r/(2*Va))+C_rda*aileron);


    //_________________________________________________________________
    /* DEFINE THE CONTROL PROBLEM */
    OCP ocp( t_start, t_end, 100 );

    ocp.minimizeMayerTerm( r*r );
    ocp.subjectTo( f );
    //ocp.subjectTo( AT_START, p_N == 0      );
    //ocp.subjectTo( AT_START, p_E == 0      );
    //ocp.subjectTo( AT_START, p_D == 150   );

    //ocp.subjectTo( AT_START, q0  == 0.9997 );
    //ocp.subjectTo( AT_START, q1  == 0      );
    //ocp.subjectTo( AT_START, q2  == 0.0232 );
    //ocp.subjectTo( AT_START, q3  == 0      );

    //ocp.subjectTo( AT_START,   u == 18     );
    //ocp.subjectTo( AT_START,   w == 0.8366 );

    //ocp.subjectTo( AT_START, elevator == 0.0079 );
    //ocp.subjectTo( AT_START, aileron == 0 );
    //ocp.subjectTo( AT_START, throttle == 0.1240 );

    //ocp.subjectTo( -1 <= elevator <= 1 );
    //ocp.subjectTo( -1 <= aileron <= 1);
    //ocp.subjectTo( -0.1 <= throttle <= 1 );


    //_________________________________________________________________
    /* PREPARE SOLUTION */

    /* Figures */
    GnuplotWindow windowStates;
    windowStates.addSubplot(p_N, "NORTH");
    windowStates.addSubplot(p_E, "EAST");
    windowStates.addSubplot(p_D, "DOWN");
	windowStates.addSubplot(Va, "AIRSPEED");
    windowStates.addSubplot(u, "U");
    windowStates.addSubplot(v, "V");

    
    /* Define algorithm */
    OptimizationAlgorithm algorithm(ocp);

    /* Solver constraints */
	algorithm.set( ABSOLUTE_TOLERANCE, 100.0 );
	algorithm.set( INTEGRATOR_TOLERANCE, 100.0 );
	//algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	//algorithm.set( MAX_NUM_ITERATIONS, 200 );
	algorithm.set( KKT_TOLERANCE, 7.0e15 );	



    algorithm << windowStates;
    algorithm.solve();

    return 0;


}
