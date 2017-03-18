/** X8 PARAMETERS: Lateral
 * Includes the lateral parameters needed for the X8 model.
 */

/** Aircraft Parameters **/
#define mass	1.56	// kg
#define J_x	    0.1147	// kg m²
#define J_y	    0.0576	// kg m²
#define J_z	    0.1712	// kg m²
#define J_xz	0.0015	// kg m²
#define S	    0.2589	// m²
#define b	    1.4224	// m
#define c	    0.3302	// m
#define S_prop	0.0314	// m²
#define rho	    1.2682	// kg/m³
#define k_motor 20
#define k_Tp	0
#define k_omega	0
#define e	    0.9

/** Longitudinal Coefficients **/
#define C_L0 	 0.09167
#define C_D0	 0.01631
#define C_m0	-0.02338
#define C_Lalp	 3.5016
#define C_Dalp	 0.2108
#define C_malp	-0.5675
#define C_Lq	 2.8932
#define C_Dq	 0
#define C_mq	-1.3990
#define C_Lde	 0.2724
#define C_Dde	 0.3045
#define C_mde	-0.3254
#define C_prop	 1.0
#define M	 50
#define alpha_0	 0.4712
#define epsilon	 0.1592
#define C_Dp	 0.0254

/** Lateral Coefficients **/
#define C_Y0	 0
#define C_l0	 0
#define C_n0	 0
#define C_Ybeta	-0.07359
#define C_lbeta	-0.02854
#define C_nbeta	-0.00040
#define C_Yp	 0
#define C_lp	-0.3209
#define C_np	-0.01297
#define C_Yr	 0
#define C_lr	 0.03066
#define C_nr	-0.00434
#define C_Yda	 0
#define C_lda	 0.1682
#define C_nda	-0.00328

/** Moments of Inertia **/
#define gamma    J_x*J_z - J_xz*J_xz
#define gamma_1  J_xz*(J_x-J_y+J_z)/gamma
#define gamma_2 (J_z*(J_z-J_y)+J_xz*J_xz)/gamma
#define gamma_3  J_z/gamma
#define gamma_4  J_xz/gamma
#define gamma_5 (J_z - J_x)/J_y
#define gamma_6  J_xz/J_y
#define gamma_7 ((J_x-J_y)*J_x+J_xz*J_xz)/gamma
#define gamma_8  J_x/gamma
