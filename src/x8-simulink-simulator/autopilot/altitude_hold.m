function theta_c = altitude_hold(h_c, h, flag,P)
  persistent integrator  differentiator  differentiator_d1  error_d1  first;
  % initialize persistent variables at beginning of simulation
  if isempty(first)
      first = 0; %Dummy variable to make sure persistent variables are initialized
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
  % variables
  Ts = P.Ts;
  tau = P.tau;
  altitude_kp = P.altitude_kp;
  altitude_ki = P.altitude_ki;
  altitude_kd = P.altitude_kd;
%   Ts = 0.01;
%   tau = 5;
%   altitude_kp = .0656;
%   altitude_ki = .0138;
%   altitude_kd = 0;
%   
  % compute the current error
  error = h_c - h;
  
  % update the integrator
  integrator = integrator + (Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator_d1...
      + (2/(2*tau+Ts))*(error - error_d1);
  
  % proportional term
  up = altitude_kp * error;
  
  % integral term
  ui = altitude_ki * integrator;
  
  % derivative term
  ud = altitude_kd * differentiator;
  
  
  % implement PID control
  if (up + ui + ud) > 30*pi/180
      theta_c = 30*pi/180;
  elseif (up + ui + ud) < -30*pi/180
      theta_c = -30*pi/180;
  else
      theta_c = up + ui + ud;
  end
  
  % implement integrator anti-windup
  if altitude_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = Ts/altitude_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end