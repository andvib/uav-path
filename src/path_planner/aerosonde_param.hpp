/** AEROSONDE PARAMETERS
 * Includes the lateral parameters needed for the Aerosonde model.
 */


/** Aircraft Parameters **/
#define mass    13.5
#define J_x     0.8244
#define J_y     1.135
#define J_z     1.759
#define J_xz    0.1204
#define S       0.55
#define b       2.8956
#define c       0.18994
#define S_prop  0.2027
#define rho     1.2682
#define k_motor 80
#define k_Tp    1e-12
#define k_omega 1.0472e03
#define e       0.9


/** Longitudinal Coefficients **/
#define C_L0    0.28
#define C_D0    0.03
#define C_m0    -0.02338
#define C_Lalp  3.45
#define C_Dalp  0.30
#define C_malp  -0.38
#define C_Lq    0
#define C_Dq    0
#define C_mq    -3.6
#define C_Lde   -0.36
#define C_Dde   0
#define C_mde   -0.5
#define C_prop  1.0
#define M       50
#define alpha_0 0.4712
#define epsilon 0.1592
#define C_Dp    0.0437
#define C_ndr   -0.032


/** Lateral Coefficients **/
#define C_Y0    0
#define C_l0    0
#define C_n0    0
#define C_Ybeta -0.98
#define C_lbeta -0.12
#define C_nbeta 0.25
#define C_Yp    0
#define C_lp    -0.26
#define C_np    0.022
#define C_Yr    0
#define C_lr    0.14
#define C_nr    -0.35
#define C_Yda   0
#define C_lda   1.08
#define C_nda   1.06
#define C_Ydr   -0.17
#define C_ldr   0.105


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

#define C_p0    gamma_3*C_l0    + gamma_4*C_n0
#define C_pbeta gamma_3*C_lbeta + gamma_4*C_nbeta
#define C_pp    gamma_3*C_lp    + gamma_4*C_np
#define C_pr    gamma_3*C_lr    + gamma_4*C_nr
#define C_pda   gamma_3*C_lda   + gamma_4*C_nda
#define C_pdr   gamma_3*C_ldr + gamma_4*C_ndr
#define C_r0    gamma_4*C_l0    + gamma_8*C_n0
#define C_rbeta gamma_4*C_lbeta + gamma_8*C_nbeta
#define C_rp    gamma_4*C_lp    + gamma_8*C_np
#define C_rr    gamma_4*C_lr    + gamma_8*C_nr
#define C_rda   gamma_4*C_lda   + gamma_8*C_nda
#define C_rdr   gamma_4*C_ldr + gamma_8*C_ndr
