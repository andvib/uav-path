#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

#include "aerosonde_param.hpp"

#define PI 3.14


int main(){

    USING_NAMESPACE_ACADO

    
    //_________________________________________________________________
    /* Introduce Variables */
    DifferentialState p_N, p_E, p_D;
    DifferentialState u, v, w;
    DifferentialState phi, theta, psi;
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

    DifferentialEquation f;


    //_________________________________________________________________
    /* AIRDATA */

    // ONLY CORRECT AS LONG AS THERE IS NO WIND
    Va = sqrt(u*u + v*v + w*w);
    alpha = atan(w/u);
    beta = asin(v/Va);    

    aero = 0.5*rho*Va*Va*S;


    //_________________________________________________________________
    /* FORCES */

    C_L = C_L0 + C_Lalp*alpha;
    C_D = C_D0 + C_Dalp*alpha;

    // Forces in x-direction
    f_gx = -mass * 9.81 * sin(theta);
    
    Cx   = -C_D*cos(alpha) + C_L*sin(alpha);
    Cxq  = - C_Dq*cos(alpha) + C_Lq*sin(alpha);
    Cxde = - C_Dde*cos(alpha) + C_Lde*sin(alpha);
    f_ax =   aero * (Cx + Cxq*(c/(2.*Va))*q + Cxde*elevator);

    f_px = 0.5*rho*S_prop*C_prop*(k_motor*k_motor*throttle*throttle - Va*Va);

    f_x  = f_gx + f_ax + f_px;

    
    // Forces in y-direction
    f_gy = mass * 9.81 * cos(theta) * sin(phi);

    f_ay = aero*(C_Y0 + C_Ybeta*beta + C_Yp*(b/(2.*Va))*p + C_Yr*(b/(2.*Va))*r + \
                                                    C_Yda*aileron + C_Ydr*rudder);

    f_y  = f_gy + f_ay;


    // Forces in z-direction
    f_gz = mass * 9.81 * cos(theta) * cos(phi);

    Cz   = -C_D*sin(alpha) - C_L*cos(alpha);
    Czq  = -C_Dq*sin(alpha) - C_Lq*cos(alpha);
    Czde = -C_Dde*sin(alpha) - C_Lde*cos(alpha);
    f_az = aero * (Cz + Czq*(c/(2.*Va))*q + Czde*elevator);

    f_z  = f_gz + f_az;


    //_________________________________________________________________
    /* MOMENTS */

    l = aero*b*(C_l0 + C_lbeta*beta + C_lp*(b/(2.*Va))*p + C_lr*(b/(2.*Va))*r + \
                                                    C_lda*aileron + C_ldr*rudder);
    
    m = aero*c*(C_m0 + C_malp*alpha + C_mq*(c/(2.*Va))*q + C_mde*elevator);
	
    n = aero*b*(C_n0 + C_nbeta*beta + C_np*(b/(2.*Va))*p + C_nr*(b/(2.*Va))*r +\
                                                    C_nda*aileron + C_ndr*rudder);


    //_________________________________________________________________
	/* DIFFERENTIAL EQUATION */

    /* Position */
	f << dot(p_N) == cos(theta)*cos(psi)*u + \
                    (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v + \
                    (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
	
    f << dot(p_E) == cos(theta)*sin(psi)*u + \
                    (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v + \
                    (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;

    f << dot(p_D) == -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;
	
    /* Speed */
    f << dot(u) == (r*v - q*w) + (1./mass)*f_x;
	f << dot(v) == (p*w - r*u) + (1./mass)*f_y;
    f << dot(w) == (q*u - p*v) + (1./mass)*f_z;
	
    /* Angles */
    f << dot(phi)   == p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    f << dot(theta) == cos(phi)*q - sin(phi)*r;
    f << dot(psi)   == (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

    /* Angle Rates */
	f << dot(p) == gamma_1*p*q - gamma_2*q*r + gamma_3*l + gamma_4*n;
    f << dot(q) == gamma_5*p*r - gamma_6*(p*p-r*r) + (1./J_y)*m;
	f << dot(r) == gamma_7*p*q - gamma_1*q*r + gamma_4*l + gamma_8*n;
	
    f << dot(elevator) == d_elevator;
    f << dot(aileron)  == d_aileron;
    f << dot(rudder)   == d_rudder;
    f << dot(throttle) == d_throttle;


    //_________________________________________________________________
    /* DEFINE LEAST SQUARE FUNCTION */

    Function h;
    DMatrix weights(2,2);  weights.setIdentity();
    DVector ref(2);        ref.setAll(0.0);

    h << p_D; ref(0) = -150.0; weights(0,0) = 2;
    h << u;   ref(1) = 35;      weights(1,1) = 4;
    

    //_________________________________________________________________
    /* Configure OCP */

    const double t_start =  0.0;
	const double t_end 	 = 10.0;
    const int    samples = 10*(t_end-t_start);

    const double kkt = 1.0e0;


    OCP ocp( t_start, t_end, samples );

    ocp.minimizeLSQ(        weights, h, ref );
    ocp.minimizeLSQEndTerm( weights, h, ref );
    ocp.subjectTo( f );

    //______________________________________________________________
    /* Start configurations */
    ocp.subjectTo( AT_START,  p_N ==  0      );
	ocp.subjectTo( AT_START,  p_E ==  0      );
    ocp.subjectTo( AT_START,  p_D == -150    );
	ocp.subjectTo( AT_START,   u  ==  35     );
	ocp.subjectTo( AT_START,   v  ==  0      );
    ocp.subjectTo( AT_START,   w  ==  0      );

	ocp.subjectTo( AT_START,  phi ==  0      );
    ocp.subjectTo( AT_START, theta==  0.0664 );
	ocp.subjectTo( AT_START,  psi ==  0      );

	ocp.subjectTo( AT_START,   p  ==  0      );
    ocp.subjectTo( AT_START,   q  ==  0      );
	ocp.subjectTo( AT_START,   r  ==  0      );

    ocp.subjectTo( AT_START, elevator == 0.1  );
    ocp.subjectTo( AT_START,  aileron == 0   );
    ocp.subjectTo( AT_START,   rudder == 0   );
    ocp.subjectTo( AT_START, throttle == 0.5 );

    ocp.subjectTo( AT_START, d_elevator == 0 );
	ocp.subjectTo( AT_START,  d_aileron == 0 );
    ocp.subjectTo( AT_START,   d_rudder == 0 );	
    ocp.subjectTo( AT_START, d_throttle == 0 );

    
    //_________________________________________________________________
    /* Constraints */

    ocp.subjectTo( -1 <= elevator <= 1 );
	ocp.subjectTo( -1 <= aileron  <= 1 );
    ocp.subjectTo( -1 <= rudder   <= 1 );	
    ocp.subjectTo(  0 <= throttle <= 1 );


    ocp.subjectTo( -0.1 <= d_elevator <= 0.1 );
    ocp.subjectTo( -0.1 <= d_aileron  <= 0.1 );
    ocp.subjectTo( -0.1 <=  d_rudder  <= 0.1 );
    ocp.subjectTo( -0.6 <= d_throttle <= 0.6 );



    //_________________________________________________________________
    /* Configure solver algorithm */
    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( INTEGRATOR_TYPE, INT_RK45 );
    algorithm.set( ABSOLUTE_TOLERANCE, 1.0e-1 );
	algorithm.set( INTEGRATOR_TOLERANCE, 1.0e-1 );
	//algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    algorithm.set( DISCRETIZATION_TYPE, SINGLE_SHOOTING );
	algorithm.set( MAX_NUM_ITERATIONS, 20 );
	algorithm.set( KKT_TOLERANCE, kkt );



    //_________________________________________________________________
    /* Prepare solution */

    GnuplotWindow windowStates;
    windowStates.addSubplot(u, "u");
    //windowStates.addSubplot(v, "v");	
    windowStates.addSubplot(p_D, "DOWN");
	windowStates.addSubplot(phi, "ROLL");
    windowStates.addSubplot(theta, "PITCH");
	windowStates.addSubplot(psi, "YAW");
    windowStates.addSubplot(elevator, "ELEVATOR");
    windowStates.addSubplot(aileron, "AILERON");
    windowStates.addSubplot(rudder, "RUDDER");
    windowStates.addSubplot(throttle, "THROTTLE");
    //windowStates.addSubplot(p, "p");
    //windowStates.addSubplot(q, "q");
    //windowStates.addSubplot(r, "r");



    GnuplotWindow windowInputRates;
    windowInputRates.addSubplot(d_elevator, "D ELEVATOR");
	windowInputRates.addSubplot(d_aileron, "D AILERON");
    windowInputRates.addSubplot(d_rudder, "D RUDDER");
	windowInputRates.addSubplot(d_throttle, "D THROTTLE");




    algorithm << windowStates;
    algorithm << windowInputRates;


	algorithm.solve();



    return 0;
}
