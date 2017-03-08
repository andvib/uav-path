close all;
clear all;

global P;
P = struct;

%param = 'aerosonde';
param = 'x8_param';
init;

%% AEROSONDE %%
%X_trim =  [-0.0000  -0.0000 -150.0000 ...
%           0.9994 -0.0009 0.0332 0.0131 ...
%           35.7334 -0.0122 2.3795...
%           0.0000 0.0000 0.0000 ...
%           -0.0300 -0.0000 -0.0295 ...
%           0.0000 0.0978]';
%U_trim = [-0.0594 0.0005 -0.0010 0.0978]';

%% X8 %%
X_trim = [0.0000    0.0000 -150.0000    0.9997    0.0000    0.0232   -0.0000 18.0168    0.0001    0.8366    0.0000    0.0000    0.0000 0.0039 -0.0000    0.0039   -0.0000    0.1240]';
U_trim = [0.0079    0.0000         0    0.1240]';

trim_states = [0 X_trim(1:13)' 0];
trim_control = [0 U_trim'];

%% Optimization problem definition
BEGIN_ACADO;

    acadoSet('problemname', 'path_crosstrack');
    
    % Define states and control inputs
    DifferentialState p_N p_E p_D;      % POSITION
    %DifferentialState phi theta psi;    % ATTITUDE
    DifferentialState q1 q2 q3 q4;      % QUATERNION
    DifferentialState u v w;            % VELOCITY
    DifferentialState p q r;            % ANGULAR VELOCITY
    DifferentialState L; % DUMMY TERM FOR TESTING

    Control elevator;
    Control aileron;
    Control rudder;
    Control throttle;
    
    % Create differential equation
    f = acado.DifferentialEquation();
    f.linkMatlabODE('uavODE');
    
    % Optimal Control Problem
    ocp = acado.OCP(0.0, 20, 40);
    ocp.minimizeMayerTerm(L); % To be replaced with crosstrack error
    
    ocp.subjectTo(f);
    
    % Initalize states
    ocp.subjectTo( 'AT_START', p_N == X_trim(1)  );
    ocp.subjectTo( 'AT_START', p_E == X_trim(2)  );
    ocp.subjectTo( 'AT_START', p_D == X_trim(3)  );
    ocp.subjectTo( 'AT_START',  q1 == X_trim(4)  );
    ocp.subjectTo( 'AT_START',  q2 == X_trim(5)  );
    ocp.subjectTo( 'AT_START',  q3 == X_trim(6)  );
    ocp.subjectTo( 'AT_START',  q4 == X_trim(7)  );
    ocp.subjectTo( 'AT_START',   u == X_trim(8)  );
    ocp.subjectTo( 'AT_START',   v == X_trim(9)  );
    ocp.subjectTo( 'AT_START',   w == X_trim(10) );
    ocp.subjectTo( 'AT_START',   p == X_trim(11) );
    ocp.subjectTo( 'AT_START',   q == X_trim(12) );
    ocp.subjectTo( 'AT_START',   r == X_trim(13) );
    ocp.subjectTo( 'AT_START',   L == 0.0        );
    
    % Initialize control
    ocp.subjectTo( 'AT_START', elevator == U_trim(1) );
    ocp.subjectTo( 'AT_START',  aileron == U_trim(2) );
    ocp.subjectTo( 'AT_START',   rudder == U_trim(3) );
    ocp.subjectTo( 'AT_START', throttle == U_trim(4) );
    
    ocp.subjectTo( 'AT_END', u == X_trim(8));
    ocp.subjectTo( 'AT_END', p_D == X_trim(3));
    ocp.subjectTo( -10  <= p_N <= 250  );
    ocp.subjectTo( -10  <= p_E <= 250  );
    ocp.subjectTo( -160 <= p_D <= -140 );
    ocp.subjectTo(  15  <=  u  <= 20   );
    ocp.subjectTo(  -1  <=  v  <= 1    );
    ocp.subjectTo(  -1  <=  w  <= 1    );
    ocp.subjectTo(  -1  <=  p  <= 1    );
    ocp.subjectTo(  -1  <=  q  <= 1    );
    ocp.subjectTo(  -1  <=  r  <= 1    );
    
    ocp.subjectTo( -1 <= elevator <= 1);
    ocp.subjectTo( -1 <= aileron  <= 1);
    ocp.subjectTo( -1 <= rudder   <= 1);
    ocp.subjectTo(  0 <= throttle <= 1);
    
    % Set optimization algorithm
    algo = acado.OptimizationAlgorithm(ocp);
    
    algo.initializeDifferentialStates(trim_states);
    algo.initializeControls(trim_control);
    
    algo.set( 'KKT_TOLERANCE', 1);
    %algo.set( 'ABSOLUTE_TOLERANCE', 1);;
    
END_ACADO;

% Run simulation
out = path_crosstrack_RUN();
draw;