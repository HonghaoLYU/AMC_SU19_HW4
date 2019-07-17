% 在Gradient Type Parameter Estimation 的基础上加上Discontinuous Projection
function [sys,x0,str,ts,simStateCompliance] = DARC_Gradient_Type_Parameter_Estimation_case3(t,x,u,flag)

dM=2;
kf=1000;
p=0.06;
Gamma=1000*diag([0.3565,0.0579,0.0069,0.0069,4.1477]);
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
    sys=mdlUpdate(t,x,u,kf,p,Gamma,Me_max,B_max,Asc_max,Acog1_max,Acog3_max,Me_min,B_min,Asc_min,Acog1_min,Acog3_min);

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
%计算我自己的初始值
theta_b_0=[B_0/Me_0;Asc_0/Me_0;Acog1_0/Me_0;Acog3_0/Me_0;1/Me_0];
% call simsizes for a sizes structure, fill it in and convert it to a sizes array.
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 5;
sizes.NumOutputs     = 5;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
% initialize the initial conditions
x0  = theta_b_0;
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

%%input um x1 x2 z2
function sys=mdlUpdate(t,x,u,kf,p,Gamma,Me_max,B_max,Asc_max,Acog1_max,Acog3_max,Me_min,B_min,Asc_min,Acog1_min,Acog3_min)
state_um=u(1);
state_x1=u(2);
state_x2=u(3);
state_z2=u(4);
theta_b_max=[B_max;Asc_max;Acog1_max;Acog3_max;1]./Me_min
theta_b_min=[B_min;Asc_min;Acog1_min;Acog3_min;1]./Me_max
Phi_b=[(-1)*state_x2 (-1)*sat(kf*state_x2) sin(state_x1*2*pi/p) sin(state_x1*6*pi/p) state_um]'
% 预测下一步的参数估计
theta_b_hat_dot=Gamma*Phi_b*state_z2
theta_b_hat_next=x+theta_b_hat_dot.*0.0005
%当预测到参数估计超出界限时，则使得参数估计维持在现有值

for i=1:5
    if theta_b_hat_next(i)>theta_b_max(i)
        theta_b_hat_dot(i)=(theta_b_max(i)-x(i))/0.0005;
    elseif theta_b_hat_next(i)<theta_b_min(i)
        theta_b_hat_dot(i)=(theta_b_min(i)-x(i))/0.0005;
    end
end
sys=x+theta_b_hat_dot.*0.0005



%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%input u x1 x2 z2 x2d_dot                              
function sys=mdlOutputs(t,x,u,kf,p,k2)

sys =x;

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
