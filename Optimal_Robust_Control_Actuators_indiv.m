%% Actuator state-space equation
Aa1 = [0 1;-wn0^2 -2*zeta0*wn0];
ba1 = [0;wn0^2];

%% LQR parameters
% Q1 = [10 0;0 1];
% Q2 = [13500 0;0 5];
% Q3 = [300 0;0 5];
% r1  = 0.5;
% Q1 = [13500 0;0 5];
% Q2 = [13500 0;0 5];%this is the gain for the hydraulic leakage fault (abrubt fault)
% Q3 = [13500 0;0 5];%
% r1  = 0.5;
Q1 = [25 0;0 1];
Q2 = [25 0;0 1];
Q3 = [25 0;0 1];
r1  = 0.5;
[Ks1,S1,P] = lqr(Aa1,ba1,Q1,r1)
[Ks2,S2,P] = lqr(Aa1,ba1,Q2,r1)
[Ks3,S3,P] = lqr(Aa1,ba1,Q3,r1)