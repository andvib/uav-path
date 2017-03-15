/** X8 PARAMETERS: Lateral
 * Includes the lateral parameters needed for the X8 model.
 */

/* Aircraft Parameters */
#define m	 1.56	// kg
#define J_x	 0.1147	// kg m²
#define J_y	 0.0576	// kg m²
#define J_z	 0.1712	// kg m²
#define J_xz	 0.0015	// kg m²
#define S	 0.2589	// m²
#define b	 1.4224	// m
#define c	 0.3302	// m
#define S_prop	 0.0314	// m²
#define rho	 1.2682	// kg/m³
#define k_motor	 20		//
#define k_Tp	 0		//
#define k_omega	 0		//
#define e	 0.9		//


/* Lateral Coefficients */
#define C_Y0	  0
#define C_l0	  0
#define C_n0	  0
#define C_Ybeta	 -0.07359
#define C_lbeta	 -0.02854
#define C_nbeta	 -0.00040
#define C_Yp	  0
#define C_lp	 -0.3209
#define C_np	 -0.01297
#define C_Yr	  0
#define C_lr	  0.03066
#define C_nr	 -0.00434
#define C_Yda	  0
#define C_lda	  0.1682
#define C_nda	 -0.00328


/* Moments of Inertia */
#define gamma    J_x*J_z - J_xz*J_xz
#define gamma_3  J_z/gamma
#define gamma_4  J_xz/gamma
#define gamma_8  J_x/gamma
