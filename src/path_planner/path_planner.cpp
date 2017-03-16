#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

#include "x8_param.hpp"

int main(){

	USING_NAMESPACE_ACADO;

	/* UAV STATES */
	DifferentialState p_N, p_E, p_D;    // Position
	DifferentialState u, v, w;	        // Speed
	DifferentialState phi, theta, psi;  // Attitude
	DifferentialState p, q, r;          // Angle rates

    /* CONTROL STATES */
    Control elevator;
    Control aileron;
    Control rudder;
    Control throttle;

    /* INTERMEDIATE STATES */
	IntermediateState f_x, f_gx, f_ax, f_px, Cx, Cxq, Cxde;
    IntermediateState f_y, f_gy, f_ay;
    IntermediateState f_z, f_gz, f_az, Cz, Czq, Czde;
	IntermediateState l, m, n;
	IntermediateState Va, alpha, beta;
    IntermediateState aero, C_D, C_L;


    /* Define Differential equation */
	DifferentialEquation f;

	const double t_start =  0.0;
	const double t_end 	 = 10.0;


    //_________________________________________________________________
    /* AIRDATA */

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
    f_ax =   aero * (Cx + Cxq*(1.0/(2.0*Va))*q + Cxde*elevator);

    f_px = 0.5*rho*S_prop*C_prop*(k_motor*k_motor*throttle*throttle - Va*Va);

    f_x  = f_gx + f_ax + f_px;

    
    // Forces in y-direction
    f_gy = mass * 9.81 * cos(theta) * sin(phi);

    f_ay = aero*(C_Y0+C_Ybeta*beta+C_Yp*(b/(2.*Va))*p+C_Yr*(b/(2.*Va))*r+C_Yda*aileron);

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

    l = aero*b*(C_l0+C_lbeta*beta+C_lp*(b/(2.*Va))*p+C_lr*(b/(2.*Va))*r+C_lda*aileron);
    
    m = aero*c*(C_m0 + C_malp*alpha + C_mq*(c/(2.*Va))*q + C_mde*elevator);
	
    n = aero*b*(C_n0+C_nbeta*beta+C_np*(b/(2.*Va))*p+C_nr*(b/(2.*Va))*r+C_nda*aileron);


    //_________________________________________________________________
	/* DIFFERENTIAL EQUATION */

    /* POSITION */
	f << dot(p_N) == cos(theta)*cos(psi)*u + \
                    (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v + \
                    (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
	
    f << dot(p_E) == cos(theta)*sin(psi)*u + \
                    (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v + \
                    (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;

    f << dot(p_D) == -sin(theta)*u + sin(phi)*cos(theta)*v + cos(phi)*cos(theta)*w;
	
    /* SPEED */
    f << dot(u) == (r*v - q*w) + (1./m)*f_x;
	f << dot(v) == (p*w - r*u) + (1./m)*f_y;
    f << dot(w) == (q*u - p*v) + (1./m)*f_z;
	
    /* ANGLES */
    f << dot(phi)   == p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    f << dot(theta) == cos(phi)*q - sin(phi)*r;
    f << dot(psi)   == (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

    /* ANGLE RATES */
	f << dot(p) == gamma_1*p*q - gamma_2*q*r + gamma_3*l + gamma_4*n;
    f << dot(q) == gamma_5*p*r - gamma_6*(p*p-r*r) + (1./J_y)*m;
	f << dot(r) == gamma_7*p*q - gamma_1*q*r + gamma_4*l + gamma_8*n;
	

    //_________________________________________________________________
	/* DEFINE THE CONTROL PROBLEM */
	OCP ocp( t_start, t_end, 10 );
	
	ocp.minimizeMayerTerm( v*v );
	ocp.subjectTo( f );
	ocp.subjectTo( AT_START,  p_N ==  0 );
	ocp.subjectTo( AT_START,  p_E ==  0 );
    ocp.subjectTo( AT_START,  p_D ==  0 );
	ocp.subjectTo( AT_START,   u  == 18 );
	ocp.subjectTo( AT_START,   v  ==  0 );
    ocp.subjectTo( AT_START,   w  ==  0 );
	ocp.subjectTo( AT_START,  phi ==  0 );
    ocp.subjectTo( AT_START, theta==  0.046 );
	ocp.subjectTo( AT_START,  psi ==  0 );
	ocp.subjectTo( AT_START,   p  ==  0 );
    ocp.subjectTo( AT_START,   q  ==  0 );
	ocp.subjectTo( AT_START,   r  ==  0 );

    ocp.subjectTo( AT_START, elevator == 0.0079 );
	ocp.subjectTo( AT_START,  aileron == 0      );
	ocp.subjectTo( AT_START,   rudder == 0      );
	ocp.subjectTo( AT_START, throttle == 0.1240 );

	//ocp.subjectTo( AT_END, u == 18 );

	//ocp.subjectTo( 15 <= u <= 22 );
	//ocp.subjectTo( -1 <= v <= 1 );

	//ocp.subjectTo( -1 <= psi <= 1 );
    //ocp.subjectTo( -1 <= theta <= 1 );
	//ocp.subjectTo( -1 <= phi <= 13);

	//ocp.subjectTo( -1 <= p <= 1 );
	//ocp.subjectTo( -1 <= r <= 1 );

	//ocp.subjectTo( -10 <= p_N <= 250 );
	//ocp.subjectTo( -10 <= p_E <= 250 );*/

    //ocp.subjectTo( -1 <= elevator <= 1 );
	//ocp.subjectTo( -1 <= aileron  <= 1 );
	//ocp.subjectTo( -1 <=  rudder  <= 1 );
	//ocp.subjectTo(  0 <= throttle <= 1 );
	

	GnuplotWindow window;
	window.addSubplot(p_E, p_N, "POSITION");
	window.addSubplot(u, "SPEED");
	window.addSubplot(psi, "HEADING");
	window.addSubplot(phi, "ROLL");
	window.addSubplot(aileron, "AILERON");
	window.addSubplot(throttle, "THROTTLE");

	/* DEFINE ALGORITHM */
	OptimizationAlgorithm algorithm(ocp);
	
	algorithm.set( ABSOLUTE_TOLERANCE, 1.0 );
	algorithm.set( INTEGRATOR_TOLERANCE, 10.0 );
	//algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	//algorithm.set( MAX_NUM_ITERATIONS, 200 );
	//algorithm.set( KKT_TOLERANCE, 1.0 );	

	algorithm << window;
	algorithm.solve();

	return 0;
}
