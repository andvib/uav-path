function delta_a = roll_hold(phi_c, phi, p, P)
  persistent integrator error_d1 first;
  % initialize persistent variables at beginning of simulation
  if isempty(first)
      first = 0; %Dummy variable to make sure persistent variables are initialized
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  %Variables
  Ts = P.Ts;
  roll_kp = P.roll_kp;
  roll_ki = P.roll_ki;
  roll_kd = 0.1*P.roll_kd;
%     roll_kp = 2.1424;
%   roll_ki = .1;
%   roll_kd = 1.2029;

  
  % compute the current error
  error = phi_c - phi;
  
  % update the integrator
  integrator = integrator + (Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = roll_kp * error;
  
  % integral term
  ui = roll_ki * integrator;
  
  % derivative term
  ud = -roll_kd*p;
  
  
  % implement PID control
  if (up + ui + ud) > 31*pi/180
      delta_a = 31*pi/180;
  elseif (up + ui + ud) < -22*pi/180
      delta_a = -22*pi/180;
  else
      delta_a = up + ui + ud;
  end
  
  % implement integrator anti-windup
  if roll_ki~=0,
    delta_a_unsat = up + ui + ud;
    k_antiwindup=Ts/roll_ki;
    integrator = integrator + k_antiwindup*(delta_a - delta_a_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
end