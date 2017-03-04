%%Highly influenced by online files from "Small unmanned aircraft" by Beard&McLain

%%
if ispc
    addpath([fileparts(mfilename('fullpath')) '\rotations']);%Contains rotation matrices
    configfile = [fileparts(mfilename('fullpath')) '\' param '.mat'];
    %addpath([fileparts(mfilename('fullpath')) '\autopilot']);%Contains autopilot
else
    addpath([fileparts(mfilename('fullpath')) '/rotations']);%Contains rotation matrices
    configfile = [fileparts(mfilename('fullpath')) param '.mat'];
    %addpath([fileparts(mfilename('fullpath')) '/autopilot']);%Contains autopilot
end

load(configfile);

P.gravity = 9.8;
     
%% Actuator dynamics
P.servoRateMax = degtorad(60)/0.11; %assumed based on standard servo performance; 60* in 0.11 sec
P.aileronRateMax = P.servoRateMax*1.1; %assumed
P.aileronMax = degtorad(31);
P.aileronMin = degtorad(-22);
P.pwmMax = 2100;
P.pwmMin = 900;

%% Matrix to convert from elevons to aileron-elevator
T_elevon2ailelev = [1 1;
                    -1 1];

%Inertia matrix: assuming symmetry wrt xz-plane -> Jxy=Jyz=0
P.I_cg =  [P.Jx,  0,  -P.Jxz;
           0      P.Jy,    0;
           -P.Jxz,0,    P.Jz];

% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_wx = 1250;
P.L_wy = 1750;
P.L_wz = 1750;
P.sigma_wx = 1; 
P.sigma_wy = 1;
P.sigma_wz = 1;
P.Va0 = 10;

% autopilot sample rate
P.Ts = 0.01;

%Initial values
lat0 = 37.6205;%37.6637;
%KSFO: 37?37.13' / W122?22.53'
lon0 = -122.3815;%-122.4778;
lat_ref = lat0;
lon_ref = lon0;
h0 = 2275.5;
h_ref = 100;
psi0 = deg2rad(113);

eta_0 = [lla2flat([lat0 lon0 h0],[lat_ref, lon_ref], rad2deg(psi0), h_ref), 0 0 psi0]';
u0 = 25;
alpha0 = -P.C_m_0/P.C_m_alpha;
ny_0 = [u0 0 u0*tan(alpha0) 0 0 0]';
x0 = [eta_0', ny_0' zeros(1,6)]';

%FlightGear parameters
%todo: fix IP initialization simIP = '78.91.4.243'; %IP address of computer running FG