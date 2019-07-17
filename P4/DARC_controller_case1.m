function [sys,x0,str,ts,simStateCompliance] = DARC_controller_case1(t,x,u,flag)

dM=2;
kf=1000;
p=0.06;
Gamma=1000*eye(7);
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
epsilon=33;
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
    sys=[];

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u,kf,p,k2,kappa,epsilon,dM,Me_0,B_0,Asc_0,Acog1_0,Acog3_0,Me_max,B_max,Asc_max,Acog1_max,Acog3_max,Me_min,B_min,Asc_min,Acog1_min,Acog3_min) ;

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
%计算我自己的初始值
% call simsizes for a sizes structure, fill it in and convert it to a sizes array.
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 9;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
% initialize the initial conditions
x0  = [];
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

%%input u x1 x2 z2 x2d_dot
function sys=mdlUpdate(t,x,u)

sys=[];


%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%input x1 x2 z2 x2d_dot 5parameters
function sys=mdlOutputs(t,x,u,kf,p,k2,kappa,epsilon,dM,Me_0,B_0,Asc_0,Acog1_0,Acog3_0,Me_max,B_max,Asc_max,Acog1_max,Acog3_max,Me_min,B_min,Asc_min,Acog1_min,Acog3_min)
state_x1=u(1);
state_x2=u(2);
state_z2=u(3);
state_x2d_dot=u(4);
theta_b_hat=u(5:9);
theta_hat=u(5:8);
b_hat=u(9);
theta_b_max=[B_max;Asc_max;Acog1_max;Acog3_max;1]/Me_min;
theta_b_min=[B_min;Asc_min;Acog1_min;Acog3_min;1]/Me_max;

Phi=[(-1)*state_x2 (-1)*sat(kf*state_x2) sin(state_x1*2*pi/p) sin(state_x1*6*pi/p)]';
b_max=1/Me_min;
b_min=1/Me_max;
um=(state_x2d_dot-Phi'*theta_hat)/b_hat;
Phi_b=[(-1)*state_x2 (-1)*sat(kf*state_x2) sin(state_x1*2*pi/p) sin(state_x1*6*pi/p) um]';
Phi_b_abs=abs(Phi_b);
h=Phi_b_abs'*(theta_b_max-theta_b_min)+dM/Me_min;
us1=(-1)*k2*state_z2/b_min;
us2=(-1)*h*tanh(kappa*h*state_z2/epsilon)/b_min;
u_darc=um+us1+us2;
sys =[u_darc;um];

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
