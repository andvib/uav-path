close all;
clear all;

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


%% Optimization problem definition
BEGIN_ACADO;

    acadoSet('problemname', 'path_crosstrack');
    
    % Define states and control inputs
    DifferentialState p_N p_E p_D;      % POSITION
    DifferentialState phi theta psi;    % ATTITUDE
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
    ocp = acado.OCP(0.0, 20, 80);
    ocp.minimizeMayerTerm(L); % To be replaced with crosstrack error
    
    ocp.initializeDifferentialStates(trim_states);
    ocp.initializeControl(trim_control);
    
    ocp.subjectTo(f);

    


END_ACADO;

% Run simulation
%out = path_crosstrack_RUN();