% 在Gradient Type Parameter Estimation 的基础上加上Discontinuous Projection
function [sys,x0,str,ts,simStateCompliance] = DIARC_Fast_Dynamic_Compensation_Type_Adaptation(t,x,u,flag)

dM=2;
kf=1000;
p=0.06;
Gamma_0=1000*eye(5);
Me_0=0.055;
B_0=0.225;
Asc_0=0.07;
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
Asc_min=0.01;
Acog1_min=0.01;
Acog3_min=0.01;
k1=10;
k2=10;
kappa=0.2785;
epsilon=5;
rho_M=2000;
gamma_d=1000;
dc_M=20;
ts_0=0.0005;
% The following outlines the general structure of an S-function.
%
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=[];

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u,gamma_d,dc_M);

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
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
% call simsizes for a sizes structure, fill it in and convert it to a sizes array.
sizes = simsizes;
% 
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
% initialize the initial conditions
x0  = [0];
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
function sys=mdlUpdate(t,x,u,gamma_d,dc_M)
state_z2=u;
dc_hat_past=x;
dc_hat_dot=gamma_d*state_z2;
dc_min=0;
% 预测下一步的估计
dc_hat_next=dc_hat_past+dc_hat_dot*0.0005;

%当预测到参数估计dc_hat超出界限时，则使得参数估计维持在现有值
if dc_hat_next>dc_M
    dc_hat_dot=(dc_M-x)/0.0005;
elseif dc_hat_next<dc_min
    dc_hat_dot=(dc_min-x)/0.0005;
end

sys=x+dc_hat_dot.*0.0005



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
