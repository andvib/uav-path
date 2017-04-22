#include <acado_optimal_control.hpp>
#include "aerosonde_param.hpp"

int main(){

    USING_NAMESPACE_ACADO


    //_________________________________________________________________
    /* Introduce Variables */
    DifferentialState p_N, p_E, h;
    DifferentialState u, v, w;
    DifferentialState phi, theta, psi;
    DifferentialState p, q, r;

    Control aileron;
    Control elevator;
    Control rudder;
    Control throttle;

    DifferentialEquation f;

    // Lateral state-space model coefficients
    double Yv, Yp, Yr, Yda, Ydr;
    double Lv, Lp, Lr, Lda, Ldr;
    double Nv, Np, Nr, Nda, Ndr;

    // Longitudinal state-space model coefficients
    double Xu, Xw, Xq, Xde, Xdt;
    double Zu, Zw, Zq, Zde;
    double Mu, Mw, Mq, Mde;

    // Misc
    const double g = 9.81;



    //_________________________________________________________________
    /* Trimmed State Variables */
    const double u_trim = 35.0;
    const double v_trim =  0.0;
    const double w_trim =  0.0;

    const double phi_trim   = 0.0;
    const double theta_trim = 0.0;
    const double psi_trim   = 0.0;

    const double p_trim = 0.0;
    const double q_trim = 0.0;
    const double r_trim = 0.0;

    const double elevator_trim = 0.0;
    const double aileron_trim = 0.0;
    const double rudder_trim = 0.0;
    const double throttle_trim = 0.0;

    const double Va_trim = 35.0;
    const double beta_trim = 0.0;
    const double alpha_trim = 0.0;


    //_________________________________________________________________
    /* Lateral State-Space model coefficients */
    Yv = ((rho*S*b*v_trim)/(4*mass*Va_trim))*(C_Yp*p_trim + C_Yr*r_trim) \
       + ((rho*S*v_trim)/mass)*(C_Y0 + C_Ybeta*beta_trim \
                                 + C_Yda*aileron_trim + C_Ydr*rudder_trim) \
       + ((rho*S*C_Ybeta)/(2*mass))*sqrt(u_trim*u_trim + w_trim*w_trim);

    Yp = w_trim + ((rho*Va_trim*S*b)/(4*mass))*C_Yp;
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

    Np = gamma_7*q_trim + ((rho*Va_trim*S*b*b)/4)*C_rp;
    Nr = -gamma_1*q_trim + ((rho*Va_trim*S*b*b)/4)*C_rr;

    Nda = ((rho*Va_trim*Va_trim*S*b)/2)*C_rda;
    Ndr = ((rho*Va_trim*Va_trim*S*b)/2)*C_rdr;


    //_________________________________________________________________
    /* Longitudinal State-Space Model Coefficients */
    Xu = ((u_trim*rho*S)/mass)*(C_X0 + C_Xalp*alpha_trim + C_Xde*elevator_trim)\
       - ((rho*S*w_trim*C_Xalp)/(2*mass)) +\
       ((rho*S*c*C_Xq*u_trim*q_trim)/(4*mass*Va_trim)) - ((rho*S_prop*C_prop*u_trim)/mass);

    Xw = -q_trim+((w_trim*rho*S)/mass)*(C_X0 + C_Xalp*alpha_trim + C_Xde*elevator_trim)\
       + ((rho*S*c*C_Xq*w_trim*q_trim)/(4*mass*Va_trim)) \
       + ((rho*S*C_Xalp*u_trim*q_trim)/(2*mass)) - ((rho*S_prop*C_prop*w_trim)/mass);

    Xq = -w_trim + ((rho*Va_trim*S*C_Xq*c)/(4*mass));
    Xde = ((rho*Va_trim*Va_trim*S*C_Xde)/(2*mass));
    Xdt = ((rho*S_prop*C_prop*k_motor*k_motor*throttle_trim*throttle_trim)/mass);

    
    Zu = q_trim+ ((u_trim*rho*S)/mass)*(C_Z0 + C_Zalp*alpha_trim + C_Zde*elevator_trim)\
       - ((rho*S*C_Zalp*w_trim)/(2*mass)) \
       + ((u_trim*rho*S*c*C_Zq*c*q_trim)/(4*mass*Va_trim));
    
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
    /* Lateral Differential Equations */
    f << dot(v) == Yv*v + Yp*p + Yr*r + g*cos(theta_trim)*cos(phi_trim) \
                 + Yda*aileron + Ydr*rudder;
    
    f << dot(p) == Lv*v + Lp*p + Lr*r + Lda*aileron + Ldr*rudder;

    f << dot(r) == Nv*v + Np*p + Nr*r + Nda*aileron + Ndr*rudder;

    f << dot(phi) == p + cos(phi_trim)*tan(theta_trim)*r   \
                   + (q_trim*cos(phi_trim)*tan(theta_trim) \
                   - r_trim*sin(phi_trim)*tan(theta_trim))*phi;

    f << dot(psi) == cos(phi_trim)*(1/cos(theta_trim))*r +
                   + (p_trim*cos(phi_trim)*(1/cos(theta_trim)) \
                   - r_trim*sin(phi_trim)*(1/cos(theta_trim)))*phi;


    //_________________________________________________________________
    /* Longitudinal Differential Equations */
    f << dot(u) == Xu*u + Xw*w + Xq*q - g*cos(theta_trim)*theta \
                                        + Xde*elevator + Xdt*throttle;

    f << dot(w) == Zu*u + Zw*w + Zq*q - g*sin(theta_trim)*theta + Zde*elevator;

    f << dot(q) == Mu*u + Mw*w + Mq*q + Mde*elevator;

    f << dot(theta) == q;

    f << dot(h) == sin(theta_trim)*u - cos(theta_trim)*w \
                    + (u_trim*cos(theta_trim)+w_trim*sin(theta_trim))*theta;






    return 0;
}
