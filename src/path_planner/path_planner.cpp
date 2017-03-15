#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <math.h>

#include "x8_param.hpp"

int main(){

	USING_NAMESPACE_ACADO;

	/* DEFINE STATES */
	DifferentialState p_N, p_E;		// Position
	DifferentialState u, v;			// Speed
	DifferentialState phi, psi;		// Attitude
	DifferentialState p, r;			// Angle rates

	IntermediateState f_x, f_y;
	IntermediateState l, n;
	IntermediateState Va;

	Control	aileron, rudder, throttle;

	DifferentialEquation f;

	const double t_start =  0.0;
	const double t_end 	 = 10.0;

	Va = sqrt(u*u + v*v);
	f_x = 0.5*rho*S_prop*1*((k_motor*k_motor*throttle*throttle) - Va*Va);
	f_y  = 0.5*rho*Va*Va*S*(C_Y0 + C_Yp*(b/(2*Va))*p + C_Yr*(b/(2*Va))*r + C_Yda*aileron);
	l = 0.5*rho*Va*Va*S*(b*(C_l0 + C_lp*(b/(2*Va))*p + C_lr*(b/(2*Va))*r + C_lda*aileron));
	n = 0.5*rho*Va*Va*S*(b*(C_n0 + C_np*(b/(2*Va))*p + C_nr*(b/(2*Va))*r + C_nda*aileron));

	//f_y = 0;
	//l = 0;
	//n = 0;

	/* DIFFERENTIAL EQUATION */
	f << dot(p_N) == cos(psi)*u - sin(psi)*cos(phi)*v;
	f << dot(p_E) == sin(psi)*u + cos(psi)*cos(phi)*v;
	f <<  dot(u)  == r*v + (1/m)*f_x;
	f <<  dot(v)  == -r*u + (1/m)*f_y;
	f << dot(phi) == p;
	f << dot(psi) == (1/cos(phi))*r;
	f <<  dot(p)  == gamma_3*l + gamma_4*n;
	f <<  dot(r)  == gamma_4*l + gamma_8*n;
	

	/* DEFINE THE CONTROL PROBLEM */
	OCP ocp( t_start, t_end, 10 );
	
	ocp.minimizeMayerTerm( phi*phi );
	ocp.subjectTo( f );
	ocp.subjectTo( AT_START, p_N ==  0 );
	ocp.subjectTo( AT_START, p_E ==  0 );
	ocp.subjectTo( AT_START,  u  == 18 );
	ocp.subjectTo( AT_START,  v  ==  0 );
	ocp.subjectTo( AT_START, phi ==  0 );
	ocp.subjectTo( AT_START, psi ==  0 );
	ocp.subjectTo( AT_START,  p  ==  0 );
	ocp.subjectTo( AT_START,  r  ==  0 );

	ocp.subjectTo( AT_START, aileron == 0 );
	ocp.subjectTo( AT_START, rudder == 0 );
	ocp.subjectTo( AT_START, throttle == 0.1240 );

	//ocp.subjectTo( AT_END, u == 18 );

	//ocp.subjectTo( 15 <= u <= 22 );
	//ocp.subjectTo( -1 <= v <= 1 );

	//ocp.subjectTo( -1 <= psi <= 1 );
	ocp.subjectTo( -1 <= phi <= 13);

	//ocp.subjectTo( -1 <= p <= 1 );
	//ocp.subjectTo( -1 <= r <= 1 );

	//ocp.subjectTo( -10 <= p_N <= 250 );
	//ocp.subjectTo( -10 <= p_E <= 250 );*/

	ocp.subjectTo( -1 <= aileron  <= 1 );
	ocp.subjectTo( -1 <=  rudder  <= 1 );
	ocp.subjectTo(  0 <= throttle <= 1 );
	//ocp.subjectTo( throttle == 0.1240 );

	GnuplotWindow window;
	window.addSubplot(p_E, p_N, "POSITION");
	window.addSubplot(Va, "SPEED");
	window.addSubplot(psi, "HEADING");
	window.addSubplot(phi, "ROLL");
	window.addSubplot(aileron, "AILERON");
	window.addSubplot(throttle, "THROTTLE");

	/* DEFINE ALGORITHM */
	OptimizationAlgorithm algorithm(ocp);
	
	algorithm.set( ABSOLUTE_TOLERANCE, 5.0 );
	algorithm.set( INTEGRATOR_TOLERANCE, 5.0 );
	algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	algorithm.set( MAX_NUM_ITERATIONS, 200 );
	algorithm.set( KKT_TOLERANCE, 1.0 );	

	algorithm << window;
	algorithm.solve();

	return 0;
}
