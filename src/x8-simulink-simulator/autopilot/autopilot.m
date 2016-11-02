function [ delta ] = autopilot( sys,P)
coder.extrinsic('warning');
%Autopilot for height, heading and speed
%Input: Current states, desired height, heading, speed, airdata
%Output: Control inputs
%System states
pn      = sys(1);
pe      = sys(2);
pd      = sys(3);
u       = sys(4);
v       = sys(5);
w       = sys(6);
phi     = sys(7);
theta   = sys(8);
psi     = sys(9);
p       = sys(10);
q       = sys(11);
r       = sys(12);

%Commanded states
Va_c    = sys(13);
h_c     = sys(14);
chi_c   = sys(15);

%limit commanded course angle to +/- 2pi
if chi_c > pi
    chi_c = chi_c - 2*pi;
elseif chi_c < -pi
    chi_c = chi_c + pi;
end

Va  = sys(16);
alpha   = sys(17);
beta    = sys(18);
w_b     = sys(19:21);
%w_ned   = Rzyx(phi,theta,psi)*w_b;
%w_n     = w_ned(1);
%w_e     = w_ned(2);
%w_d     = w_ned(3);

%Other states
h       = -pd;
%chi     = atan2(Va*sin(psi)+w_e, Va*cos(psi)+w_n);
chi     = sys(22);
%Variables
altitude_take_off_zone = 10;
altitude_hold_zone = 5;


%% Lateral autopilot    
delta_r = 0;
phi_c   = course_hold(chi_c, chi, r,P);
delta_a = roll_hold(phi_c, phi, p, P);

%% Longitudinal autopilot
% define persistent variable for state of altitude state machine
persistent altitude_state initialize_integrator first_long;
% initialize persistent variable
if isempty(first_long)
    first_long = 0; %Dummy variable to make sure persistent variables are initialized
    if h<=altitude_take_off_zone,     
        altitude_state = 1;
    elseif h<=h_c-altitude_hold_zone, 
        altitude_state = 2;
    elseif h>=h_c+altitude_hold_zone, 
        altitude_state = 3;
    else
        altitude_state = 4;
    end
    initialize_integrator = 1;
end

% implement state machine
switch altitude_state,
    case 1,  % in take-off zone
        delta_t = 1;
        theta_c = 30*pi/180;
        if h>=altitude_take_off_zone,
            altitude_state = 2;
            initialize_integrator = 1;
        else
            initialize_integrator = 0;
        end

    case 2,  % climb zone
        delta_t = 1;
        theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator,P);
        if h>=h_c-altitude_hold_zone,
            altitude_state = 4;
            initialize_integrator = 1;
        elseif h<=altitude_take_off_zone,
            altitude_state = 1;
            initialize_integrator = 1;
        else
            initialize_integrator = 0;
        end

    case 3, % descend zone
        delta_t = 0;
        theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator,P);
        if h<=h_c+altitude_hold_zone,
            altitude_state = 4;
            initialize_integrator = 1;
        else
            initialize_integrator = 0;
        end
    case 4, % altitude hold zone
        delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator,P);
        theta_c = altitude_hold(h_c, h, initialize_integrator,P);
        if h<=h_c-altitude_hold_zone,
            altitude_state = 2;
            initialize_integrator = 1;
        elseif h>=h_c+altitude_hold_zone,
            altitude_state = 3;
            initialize_integrator = 1;
        else
            initialize_integrator = 0;
        end
     otherwise
         theta_c = 0;
         warning('Altitudestate switch in autopilot went to otherwise');
         delta_t = 0;
end

delta_e = pitch_hold(theta_c, theta, q, P);

%% Output
delta = [delta_e;delta_a;delta_r;delta_t];
end

