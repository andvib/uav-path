function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag,P)
  persistent integrator differentiator differentiator_d1 error_d1 first;
  % initialize persistent variables at beginning of simulation
  if isempty(first)
      first = 0; %Dummy variable to make sure persistent variables are initialized
      integrator = 0; 
      differentiator = 0;
      differentiator_d1 = 0;
      error_d1   = 0; 
  end
  % Variables
  Ts = P.Ts;
  tau = P.tau;
  airspeed_throttle_kp = P.airspeed_throttle_kp;
  airspeed_throttle_ki = P.airspeed_throttle_ki;
  airspeed_throttle_kd = P.airspeed_throttle_kd;
  u_trim = P.u_trim(4);
%   tau = 5;
%   airspeed_throttle_kp = 7.0497;
%   airspeed_throttle_ki = 0.9649;
%   airspeed_throttle_kd = 0;
%   u_trim = 0.9135;
  
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator_d1...
      + (2/(2*tau+Ts))*(error - error_d1);
  
  % proportional term
  up = airspeed_throttle_kp * error;
  
  % integral term
  ui = airspeed_throttle_ki * integrator;
  
  % derivative term
  ud = airspeed_throttle_kd * differentiator;  
  
  % implement PID control
  if (u_trim+up + ui + ud) > 1
      delta_t = 1;
  elseif (u_trim+up + ui + ud) < 0
      delta_t = 0;
  else
      delta_t = u_trim+up + ui + ud;
  end
  
  % implement integrator anti-windup
  if airspeed_throttle_ki~=0,
    delta_t_unsat = u_trim + up + ui + ud;
    k_antiwindup = Ts/airspeed_throttle_ki;
    integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end