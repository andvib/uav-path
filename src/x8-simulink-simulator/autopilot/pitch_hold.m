function delta_e = pitch_hold(theta_c, theta, q, P)
  persistent integrator error_d1 first;
  % initialize persistent variables at beginning of simulation
  if isempty(first)
      first = 0; %Dummy variable to make sure persistent variables are initialized
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
  % Variables
    Ts = P.Ts;
  pitch_kp = P.pitch_kp;
  pitch_ki = P.pitch_ki;
  pitch_kd = P.pitch_kd;
%   Ts = 0.01;
%   pitch_kp = -.5477;
%   pitch_ki = 0;
%   pitch_kd = -.1208;
  
  % compute the current error
  error = theta_c - theta;
  
  % update the integrator
  integrator = integrator + (Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = pitch_kp * error;
  
  % integral term
  ui = pitch_ki * integrator;
  
  % derivative term
  ud = -pitch_kd * q;
  
  
  % implement PID control
  if (up + ui + ud) > 35*pi/180
      delta_e = 35*pi/180;
  elseif (up + ui + ud) < -30*pi/180
      delta_e = -30*pi/180;
  else
      delta_e = up + ui + ud;
  end
  
  % implement integrator anti-windup
  if pitch_ki~=0,
    delta_e_unsat = up + ui + ud;
    k_antiwindup = Ts/pitch_ki;
    integrator = integrator + k_antiwindup*(delta_e-delta_e_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end