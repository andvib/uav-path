function phi_c = course_hold(chi_c, chi, r, P)
  persistent integrator error_d1 first;
  % initialize persistent variables at beginning of simulation
  if isempty(first)
      first = 0; %Dummy variable to make sure persistent variables are initialized
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
  
  % Variables
  Ts = P.Ts;
%   course_kp = .7257;
%   course_ki = .1344;
%   course_kd = 0;
   course_kp = P.course_kp;
  course_ki = P.course_ki;
  course_kd = P.course_kd;
  % compute the current error
  error = chi_c - chi;
  
  % update the integrator
  if abs(error)>15*pi/180,
      integrator = 0;
  else
      integrator = integrator + (Ts/2)*(error + error_d1); % trapazoidal rule
  end
  
  % proportional term
  up = course_kp * error;
  
  % integral term
  ui = course_ki * integrator;
  
  % derivative term
  ud = -course_kd*r;
  
  
  % implement PID control
  if (up + ui + ud) > 45*pi/180
      phi_c = 45*pi/180;
  elseif (up + ui + ud) < -45*pi/180
      phi_c = -45*pi/180;
  else
      phi_c = up + ui + ud;
  end
  
  % implement integrator anti-windup
  if course_ki~=0,
    phi_c_unsat = up+ui+ud;
    k_antiwindup = Ts/course_ki;
    integrator = integrator + k_antiwindup*(phi_c-phi_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
end