function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag,P)
  persistent integrator differentiator differentiator_d1   error_d1 first;
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
  airspeed_pitch_kp = P.airspeed_pitch_kp;
  airspeed_pitch_ki = P.airspeed_pitch_ki;
  airspeed_pitch_kd = P.airspeed_pitch_kd;
%   airspeed_pitch_kp = -0.2770;
%   airspeed_pitch_ki = -0.3373;
%   airspeed_pitch_kd = 0;
  
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator_d1...
      + (2/(2*tau+Ts))*(error - error_d1);
  
  % proportional term
  up = airspeed_pitch_kp * error;
  
  % integral term
  ui = airspeed_pitch_ki * integrator;
  
  % derivative term
  ud = airspeed_pitch_kd * differentiator;
  
  
  % implement PID control
  if (up + ui + ud) > 35*pi/180
      theta_c = 35*pi/180;
  elseif (up + ui + ud) < -35*pi/180
      theta_c = -35*pi/180;
  else
      theta_c = up + ui + ud;
  end
  
  % implement integrator antiwindup
  if airspeed_pitch_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = Ts/airspeed_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
  differentiator_d1 = differentiator;

end