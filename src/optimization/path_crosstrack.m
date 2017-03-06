close all;
clear all;

global P north east;
P = struct;

param = 'aerosonde';
init;

X_trim =  [-0.0000  -0.0000 -150.0000 ...
           0.9994 -0.0009 0.0332 0.0131 ...
           35.7334 -0.0122 2.3795...
           0.0000 0.0000 0.0000 ...
           -0.0300 -0.0000 -0.0295 ...
           0.0000 0.0978]';
U_trim = [-0.0594 0.0005 -0.0010 0.0978]';
Vad = 35;

trim_states = [0 X_trim(1:13)' 0];
trim_control = [0 U_trim'];

north = [];
east = [];

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
    ocp = acado.OCP(5.0, 10.0, 40);
    ocp.minimizeMayerTerm(L); % To be replaced with crosstrack error
    
    ocp.subjectTo(f);
    %ocp.subjectTo( 'AT_END', 35 <= u );
    %ocp.subjectTo( 'AT_END', u <= 36);
    %ocp.subjectTo( 0 <= p_N );
    %ocp.subjectTo( p_N <= 50 );
    %ocp.subjectTo( 0 <= p_E );
    %ocp.subjectTo( p_E <= 50 );
    %ocp.subjectTo( 34 <= u );
    %ocp.subjectTo( u <= 36 );
    
    % Set optimization algorithm
    algo = acado.OptimizationAlgorithm(ocp);
    
    algo.initializeDifferentialStates(trim_states);
    algo.initializeControls(trim_control);
    
    algo.set( 'KKT_TOLERANCE', 1e-5);
    algo.set( 'ABSOLUTE_TOLERANCE', 1e-5);
    
END_ACADO;

% Run simulation
out = path_crosstrack_RUN();