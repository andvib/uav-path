%%  forces.m
%   Function called from Simulink that calculates the gravitational,
%   aerodynamical and propulsional forces acting on the UAS. Based on
%   chapter 4 in Beard&McLain
%   
%   Following SNAME naming convention
%   input:
%   output: F,m,
%
%   krisgry 03.09.14
%%

function [output] = forces(eta,ny,quat,control,wind,P)
% relabel the inputs
%     eta
%     ny
%     control: elevator, aileron, throttle
%     wind: wind components in N E D, rates in pqr
%     pos     = eta(1:3);
    Theta   = eta(4:6);
    vel     = ny(1:3);
    rate    = ny(4:6);
    
%     N       = pos(1);
%     E       = pos(2);
%     D       = pos(3);
%     u       = vel(1);
%     v       = vel(2);
%     w       = vel(3);
    phi     = Theta(1);
    theta   = Theta(2);
    psi     = Theta(3);
    p       = rate(1) + wind(4);
    q       = rate(2) + wind(5);
    r       = rate(3) + wind(6);
    
    elevator= control(1);
    aileron = control(2);
    rudder  = control(3);
    throttle= control(4);
       
    %% relative velocity
    wind_b = wind(1:3);
    vel_r = vel-wind_b;                               
    u_r = vel_r(1);
    v_r = vel_r(2);
    w_r = vel_r(3);
    
    %% compute airspeed Va, angle-of-attack alpha, side-slip beta. See (2.8)
    % in B&M
    Va    = sqrt(u_r^2+v_r^2+w_r^2);
    if(Va == 0)
        Va = 0.00001;
    end

    alpha = atan2(w_r,u_r);
    beta  = asin(v_r/Va);
    
    %% Compute crab angle, course angle and flight path angle
    gamma_a = theta - alpha;
    gamma   = theta - atan2(vel(3),vel(1));
    vel_n = Rquat(quat)*vel;
    course = atan2(vel_n(2),vel_n(1));
    crab = course - psi;
 
    %% compute gravitational force
    fg_v = [0;0;P.mass*P.gravity];                  %gravity in NED frame
    F_g = Rquat(quat)'*fg_v;                %body frame
    
    %% Longitudinal mode
    %nonlinear lift coefficient
    sigma = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))...
           /((1 + exp(-P.M*(alpha-P.alpha0)))*(1 + exp(P.M*(alpha+P.alpha0))));
    C_L_alpha = (1-sigma)*(P.C_L_0 + P.C_L_alpha*alpha) + sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    % compute aero lift force
    f_lift_s = 0.5*P.rho*Va^2*P.S_wing*(C_L_alpha + P.C_L_q*P.c/(2*Va)*q + P.C_L_delta_e*elevator);     %eqn 4.6 in B&M
    
    % drag
    C_D_alpha = P.C_D_p + (1-sigma)*(P.C_L_0 + P.C_L_alpha*alpha)^2/(pi*P.e*(P.b^2/P.S_wing)) + sigma*2*sign(alpha)*sin(alpha)^3;
    %eqn 4.7 in B&M
    f_drag_s = 0.5*P.rho*Va^2*P.S_wing*(C_D_alpha + P.C_D_q*P.c/(2*Va)*q + P.C_D_delta_e*abs(elevator) + P.C_D_beta2*beta^2);     %linear and constant terms in beta are zero
    
    % pitch moment
%     m_a = (P.C_m_0 + (P.C_m_alpha)*alpha);
    m_a = ((1-sigma)*(P.C_m_0 + P.C_m_alpha*alpha) + sigma*(P.C_m_flatplate*sign(alpha)*sin(alpha)^2));
    m_q = (P.C_m_q)*P.c/(2*Va)*q;
    m_elev = P.C_m_delta_e*elevator; %eqn 4.5 in B&M
    m = 0.5*P.rho*Va^2*P.S_wing*P.c*(m_a + m_q + m_elev);
    
    %% Lateral mode
    f_y = 0.5*P.rho*Va^2*P.S_wing*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/(2*Va)*p + P.C_Y_r*P.b/(2*Va)*r + P.C_Y_delta_a*aileron + P.C_Y_delta_r*rudder); %eqn 4.14 in B&M
    
    l   = 0.5*P.rho*Va^2*P.b*P.S_wing*(P.C_ell_0 + P.C_ell_beta*beta + ((P.C_ell_p+0.1)*p-0.1*p^3)*P.b/(2*Va) + P.C_ell_r*P.b/(2*Va)*r + P.C_ell_delta_a*aileron + P.C_ell_delta_r*rudder); %eqn 4.15
%     l   = 0.5*P.rho*Va^2*P.b*P.S_wing*(P.C_ell_0 + P.C_ell_beta*beta + (P.C_ell_p*p)*P.b/(2*Va) + P.C_ell_r*P.b/(2*Va)*r + P.C_ell_delta_a*aileron + P.C_ell_delta_r*rudder); %eqn 4.15
    
    n_b = (P.C_n_0 + P.C_n_beta*beta);
    n_p = (P.C_n_p*P.b/(2*Va)*p);
    n_r = (P.C_n_r*P.b/(2*Va)*r);
    n_da= (P.C_n_delta_a*aileron);
    n_dr= (P.C_n_delta_r*rudder);
    n   = 0.5*P.rho*Va^2*P.b*P.S_wing*(n_b + n_p + n_r + n_da + n_dr); %eqn 4.16
    
    %% Sum aero    %Convert from s to b frame
    F_aero = Ry(alpha)'*[-f_drag_s; 0; -f_lift_s] + [0; f_y; 0];
    T_aero = [l; m; n];
          
    %% Prop force
    Vd = Va + throttle*(P.k_motor-Va); %Discharge velocity
    F_prop = [0.5*P.rho*P.S_prop*P.C_prop*Vd*(Vd-Va); 0; 0]; %B&M/ Calculation of thrust in a ducted fan assembly for hovercraft
%     F_prop = 0.5*P.rho*P.S_prop*P.C_prop*[(P.k_motor*throttle)^2 - Va^2; 0; 0];
    T_prop = [-P.k_T_P*(P.k_Omega*throttle)^2;0;0];
    
    %% Sum forces
    Force = F_prop + F_g + F_aero;
    Torque = T_aero + T_prop;
    
    output = [Force; Torque; Va; alpha; beta; course; crab; gamma; gamma_a; wind_b;T_prop(1)];
end
