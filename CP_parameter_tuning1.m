
clc
%% Define variables
% PitchActuator_Faults_run  % getting the WT parameters

CP_0     = 2*P0/(rho*Area*v0^3); % CP at rate condition
% sim('OpenLoop_ModelVerification');
% CP_temp = sigsOut.getElement('CP').Values.Data;
% time_vec = sigsOut.getElement('CP').Values.Time;
% CP_0 = median(CP_temp);
% figure, plot(time_vec,CP_temp, time_vec, CP_0*ones(size(time_vec))); %examine goodness of steady state extraction


lambda_0 = v0/w0;           % THIS IS NOT TIP-SPEED RATIO. Just a simplifying definition
n2_0     = exp(-m2*lambda_0);


%% Optimization problem
% Probelem setup
% X = [m1;n2;m3]
X0 = [m1;n2_0;m3];
Obj_fun = @(X) norm(X-X0)^2;
Nonlcon = @(X) deal([],(lambda_0-X(1)-X(3)*Beta0^2)*X(2) - 2*CP_0);

LB      = [-inf;3e-4;0];  % lower bound

X = fmincon(Obj_fun,X0,[],[],[],[],LB,[],Nonlcon);

%% updating the parameters
m1 = X(1)
m2 = -log(X(2))/lambda_0
m3 = X(3)
