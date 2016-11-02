function [A,B,C,D,A_lat,B_lat,A_lon,B_lon,X_trim,U_trim,Y_trim,DX,tf] = trimX8(bode,Va,gamma,R,h_ini,phi0)
%%Find the trim conditions for X8

%% Define State Values
%
if nargin < 5
    if nargin < 4
        if nargin < 3
            if nargin < 2
                Va = 18.0;
            end
            gamma = 0.0;
        end
        R=Inf;
    end
    h_ini = 100;      % Trim Height [m]
end
tf = 0; %Variable used to return transfer functions if desired

%% Find Names and Ordering of States from Simulink(R) Model
%
[sizes,x0,names]=X8_trim([],[],[],0);

%number of constraints should equal number of states to avoid rank deficiency in trim algorithm?

%% Trim Model
% Set initial conditions for states, derivatives, inputs and outputs
% indicate which states/derivatives/inputs/outputs should be fixed 
% to the initial conditions
%                  e a r t
u0              = [-0.064 0.1 0 0.1]'; %Avoid negative throttle by guessing thr=1
fixed_inputs    = [];

theta0=gamma;psi0=0;
quat0 = euler2q(phi0,theta0,psi0);
%                   N E    D           q1       q2       q3         q4          u   v  w   p   q   r  servoRpos       servoRrate servoLpos        servoLrate   motor
%                   1 2    3            4       5         6         7           8   9  10  11  12  13  14               15        16                17          18
x0              =  [0 0 -h_ini         quat0(1) quat0(2) quat0(3)   quat0(4)    Va  0  0   0   0   0  (u0(1)-u0(2))/2      0     (u0(1)+u0(2))/2       0       u0(4)]';
dx0             =  [0 0 -Va*sin(gamma) 0        0           0       0           0   0  0   0   0   0    0                  0       0                   0       0]';
if R~=Inf,dx0(6)= Va/R; end %psidot = Va/R
fixed_states    =  [];%14:17 18];
fixed_deriv     =  [       3           4     5   6  7  8  9  10   11  12 13 14:17 18];

%                  Va alpha beta
y0              = [Va gamma 0   x0(1:13)' phi0,theta0,psi0]';
fixed_outputs   = [1,3];%, 4:19];

% Calculate trim conditions
options(14) = 10000;
[X_trim,U_trim,Y_trim,DX,options]=trim('X8_trim',x0,u0,y0, ...
                               fixed_states,fixed_inputs,fixed_outputs, ...
                               dx0,fixed_deriv,options);  %#ok<NOPTS>

X_trim(4:7) = quatnormalize(X_trim(4:7)')'
% Find linearized model around trim conditions
[A,B,C,D]=linmod('X8_trim',X_trim,U_trim);

if exist('control','dir')
  %airframe = ss(A(n_deriv,n_deriv),B(n_deriv,:),C([2 1],n_deriv),D([2 1],:));
  airframe = ss(A,B,C,D);
%   [a,b,c,d] = ssdata(minreal(airframe));
%   size(a);
  set(airframe,'StateName',[{'N'} {'E'} {'D'} {'q1'} {'q2'} {'q3'} {'q4'} {'u'} {'v'} {'w'} {'p'} {'q'} {'r'} {'servoR_pos'} {'servoR_vel'} {'servoL_pos'} {'servoL_vel'} {'prop_speed'}]', ...
               'InputName',[{'Elevator'} {'Aileron'} {'Rudder'} {'Throttle'}], ...
               'OutputName',[{'Va'} {'alpha'} {'beta'} {'N'} {'E'} {'D'} {'q1'} {'q2'} {'q3'} {'q4'} {'u'} {'v'} {'w'} {'p'} {'q'} {'r'} {'phi'} {'theta'} {'psi'}]);

  tf = zpk(airframe);
  if bode
      ltiview('bode',airframe);    
  end
end

%% Transform into lateral and longitudinal
% x_lat =[v p r phi psi]; u_lat=[aileron]
% % x_lon =[u w q theta h]; u_lon=[elevator, throttle]
% T_lat = [   0 0 0 0 0 0 0 1 0 0 0 0;
%             0 0 0 0 0 0 0 0 0 1 0 0;
%             0 0 0 0 0 0 0 0 0 0 0 1;
%             0 0 0 1 0 0 0 0 0 0 0 0;
%             0 0 0 0 0 1 0 0 0 0 0 0;];
% T2_lat = [  0 1 1 0];
A_lat = 0;%T_lat*A*T_lat';
B_lat = 0;%T_lat*B*T2_lat';
% %         
% T_lon = [   0 0  0 0 0 0 1 0 0 0 0 0;
%             0 0  0 0 0 0 0 0 1 0 0 0;
%             0 0  0 0 0 0 0 0 0 0 1 0;
%             0 0  0 0 1 0 0 0 0 0 0 0;
%             0 0 -1 0 0 0 0 0 0 0 0 0;];
% T2_lon = [  1 0 0 0;
%             0 0 0 1];
A_lon = 0;%T_lon*A*T_lon';
B_lon = 0;%T_lon*B*T2_lon';
end

