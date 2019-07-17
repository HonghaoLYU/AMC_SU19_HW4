function [sys,x0,str,ts,simStateCompliance] = Gradient_Type_Parameter_Estimation(t,x,u,flag)
dM=2;
kf=1000;
p=0.06;
Gamma=100*eye(6);
Me_0=0.055;
B_0=0.225;
Asc_0=0.125;
Acog1_0=0.03;
Acog3_0=0.03;
d0_0=0;
k1=10;
k2=10;
ts_0=0.0005;
% The following outlines the general structure of an S-function.
switch flag
    case 0
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Me_0,B_0,Asc_0,Acog1_0,Acog3_0,d0_0);
    case 1
        sys=[];
    case 2
        sys=mdlUpdate(t,x,u,kf,p,Gamma);
    case 3
        sys=mdlOutputs(t,x,u,kf,p,k2) ;
    case 4
        sys=[];
    case 9
        sys=[];
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(Me_0,B_0,Asc_0,Acog1_0,Acog3_0,d0_0)
% Calculate my own initial value
theta0=[B_0/Me_0;Asc_0/Me_0;Acog1_0/Me_0;Acog3_0/Me_0;1/Me_0;d0_0/Me_0];
% call simsizes for a sizes structure, fill it in and convert it to a sizes array.
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
% initialize the initial conditions
x0  = theta0
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

%%input u x1 x2 z2
function sys=mdlUpdate(t,x,u,kf,p,Gamma)
Phi_bd=[(-1)*u(3) (-1)*sat(kf*u(3)) sin(u(2)*2*pi/p) sin(u(2)*6*pi/p) u(1) 1]'
sys=x+Gamma*Phi_bd*u(4)*0.0005

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
    f=1
elseif para<-1
    f=-1
else
    f=para;
end