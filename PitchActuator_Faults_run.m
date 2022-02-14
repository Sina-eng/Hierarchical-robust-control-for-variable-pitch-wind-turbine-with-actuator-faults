load('Test18.mat')
Ts    = 0.00625;          %sample time
fc    = 0.25;             %low pass corner frequency
Alpha = exp(-2*pi*Ts*fc); %low pass filter coefficient 
%% CP params
m1 = 5.6;          %coefficient in the CP equatin
m2 = 0.17;         %coefficient in the CP equatin
m3 = 72.22;        %coefficient in the CP equatin

%% Initialization at the rated vallues
v0 = 22;           %rated operating wind speed [m/s] 
P0 = 5296610;      %rated electrical power     [watt]
w0 = 12.1*pi/30;   %rated rotor speed          [rad/s]
omega_0 = w0;
v_max=25;          %maximum rated wind speed   [m/s]
Beta0 = 19.94;     %operating pitch angle [deg]
rho   = 1.225;     %air density
Area  =(pi/4)*(126*cosd(2.5))^2; %rotor swept area


%% Actuator Parameters
PC_MaxPit = 1.570796*180/pi;   %maximum pitch setting
PC_MaxRat = 0.1396263*180/pi;  %maximum pitch rate (in absolute value)
PC_MinPit = 0.0;

n           =  3;                %Number of actuators
zeta        =  0.6;              %damping ratio at free fault
wn          =  11.11;            %The natural frequency at free fault
zeta_0      =  0.6;              %damping ratio at free fault
wn_0        =  11.11;            %The natural frequency at free fault
wn0         =  wn_0;
zeta0       =  zeta_0;
%% Mechanical + Aerodaynamic Parameters
P0  = 5296610;       % Rated Power [Watt]
J   = 43784700;                  % rotor inertia     [kg-m^2]
rho = 1.225;                     % air density       [kgm^-3]
A   = (pi/4)*(126*cosd(2.5))^2;  % rotor sweep area  [m^2]
R   = sqrt(A/pi);                %radius of swept area
Ng  = 97;                        % gear ratio        [dimensionless]
c   =    rho*A*R;                %coeffient in the aerodynamic torque model
CP_parameter_tuning1
CP_params = [m1;m2;m3];          %constant parameters in the aerodynamics torque


%% High-level loop parameters
Beta_max =  90;                  %maximum pitch angle [deg]
phi_0    =  3*Beta0^2;           %nominal feedforward [deg^2]
phi_bar  =  3*Beta_max^2;        %maximum control authority [deg^2]


%% Faulty parameters
%leakage (drop pressure) (fault 6)
wn2   = 5.73;  %faulty natural frequency
zeta2 = 0.45;  %faulty damping ratio

%high air content (fault 7)
wn3   = 3.42;  %faulty natural frequency 
zeta3 = 0.9;   %faulty damping ratio
%%Fault models
%transfers to ss models
[Apb,Bpb,Cpb,Dpb]     = tf2ss([wn_0^2],[1 2*zeta_0*wn_0 wn_0^2]); %nominal model
[Apb1,Bpb1,Cpb1,Dpb1] = tf2ss([wn2^2],[1 2*zeta2*wn2 wn2^2]);     %hydraulic leakge state space faulty model
[Apb2,Bpb2,Cpb2,Dpb2] = tf2ss([wn3^2],[1 2*zeta3*wn3 wn3^2]);     %high-air content state space faulty model
Optimal_Robust_Control_Actuators_indiv

%% High level control parameters
ki = 1000;    %alpha=0.4 with gamma=1
% ki = 500;  %alpha=0.2 with gamma=0.9
% ki = 200;  %alpha=0.08 with gamma=0.85
kp = 2500;   %k