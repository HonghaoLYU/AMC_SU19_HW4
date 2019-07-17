% 在Gradient Type Parameter Estimation 的基础上加上Discontinuous Projection
function [sys,x0,str,ts,simStateCompliance] = IARC_Least_Square_Parameter_Estimation(t,x,u,flag)

dM=2;
kf=1000;
p=0.06;
Gamma_0=1000*eye(5);
Me_0=0.055;
B_0=0.225;
Asc_0=0.125;
Acog1_0=0.03;
Acog3_0=0.03;
d0_0=0;
Me_max=0.085;
B_max=0.35;
Asc_max=0.15;
Acog1_max=0.05;
Acog3_max=0.05;
Me_min=0.025;
B_min=0.1;
Asc_min=0.1;
Acog1_min=0.01;
Acog3_min=0.01;
k1=10;
k2=10;
kappa=0.2785;
epsilon=5;
rho_M=2000;
ts_0=0.0005;
% The following outlines the general structure of an S-function.
%
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Me_0,B_0,Asc_0,Acog1_0,Acog3_0);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=[];

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u,kf,p,rho_M,Me_max,B_max,Asc_max,Acog1_max,Acog3_max,Me_min,B_min,Asc_min,Acog1_min,Acog3_min);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u,kf,p,k2) ;

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=[];

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=[];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Me_0,B_0,Asc_0,Acog1_0,Acog3_0)
%计算参数的初始值
theta_b_0=[B_0/Me_0;Asc_0/Me_0;Acog1_0/Me_0;Acog3_0/Me_0;1/Me_0];
x_6_20_0=[1000;0;0;0;0;1000;0;0;0;1000;0;0;1000;0;1000];
% call simsizes for a sizes structure, fill it in and convert it to a sizes array.
sizes = simsizes;
%           [x6  x7  x8   x9 x10]
%           [x7  x11 x12 x13 x14]
% Gamma(t)= [x8  x12 x15 x16 x17]
%           [x9  x13 x16 x18 x19]
%           [x10 x14 x17 x19 x20]
% 

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 20;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
% initialize the initial conditions
x0  = [theta_b_0;x_6_20_0];
% str is always an empty matrix
str = [];
% initialize the array of sample times
ts  = [0.0005 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================

function sys=mdlDerivatives(t,x,u)
sys=[];

%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================

%%input Phi_bf;Hf[x2_dot]
function sys=mdlUpdate(t,x,u,kf,p,rho_M,Me_max,B_max,Asc_max,Acog1_max,Acog3_max,Me_min,B_min,Asc_min,Acog1_min,Acog3_min)
Phi_bf=u(1:5);
Hf_x2_dot=u(6);
theta_b_max=[B_max;Asc_max;Acog1_max;Acog3_max;1]./Me_min
theta_b_min=[B_min;Asc_min;Acog1_min;Acog3_min;1]./Me_max
% 预测下一步的参数估计
theta_b_hat_past=x(1:5);
error_pre=Phi_bf'*theta_b_hat_past;
Gamma_past=[x(6)  x(7)  x(8)   x(9) x(10);
            x(7)  x(11) x(12) x(13) x(14);
            x(8)  x(12) x(15) x(16) x(17);
            x(9)  x(13) x(16) x(18) x(19);
            x(10) x(14) x(17) x(19) x(20)]
% 计算下一时刻的Gamma
mu_1=0.1;
mu_2=1.5
D_Gamma=mu_1*Gamma_past-mu_2*Gamma_past*(Phi_bf*Phi_bf')*Gamma_past;
Gamma_dot=D_Gamma;
Gamma_next=Gamma_past+Gamma_dot*0.0005;
% 当Gamma（t）的最大特征值超出界限时，维持在现有值
lambda=eig(Gamma_next);
lambda_max=max(lambda);
if lambda_max>rho_M
    Gamma_dot=zeros(5)
end
% 计算下一时刻的theta
theta_b_hat_dot=Gamma_past*Phi_bf*error_pre;
theta_b_hat_next=x(1:5)+theta_b_hat_dot.*0.0005;
%当预测到参数估计theta超出界限时，则使得参数估计维持在现有值
for i=1:5
    if theta_b_hat_next(i)>theta_b_max(i)
        theta_b_hat_dot(i)=(theta_b_max(i)-x(i))/0.0005;
    elseif theta_b_hat_next(i)<theta_b_min(i)
        theta_b_hat_dot(i)=(theta_b_min(i)-x(i))/0.0005;
    end
end
% Gamma_past=[x(6)  x(7)  x(8)   x(9) x(10);
%             x(7)  x(11) x(12) x(13) x(14);
%             x(8)  x(12) x(15) x(16) x(17);
%             x(9)  x(13) x(16) x(18) x(19);
%             x(10) x(14) x(17) x(19) x(20)]
% 计算状态更新量
state_x_dot=[theta_b_hat_dot;Gamma_dot(:,1);Gamma_dot(2:5,2);Gamma_dot(3:5,3);Gamma_dot(4:5,4);Gamma_dot(5,5)];
sys=x+state_x_dot.*0.0005



%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%input u x1 x2 z2 x2d_dot                              
function sys=mdlOutputs(t,x,u,kf,p,k2)

sys =x(1:5);

%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
function f=sat(para)
if para>1
    f=1;
elseif para<-1
    f=-1;
else
    f=para;
end
