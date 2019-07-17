function [sys,x0,str,ts,simStateCompliance] = Plant_case3(t,x,u,flag)

Me=0.025;
B=0.1;
Asc=0.1;
Acog1=0.01;
Acog3=0.05;

dM=2;
kf=1000;
p=0.06;
k1=10;

% The following outlines the general structure of an S-function.
switch flag
    case 0
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    case 1
        sys=mdlDerivatives(t,x,u,kf,Me,B,Asc,Acog1,Acog3,p);
    case 2
        sys=[];
    case 3
        sys=mdlOutputs(t,x,u);
    case 4
        sys=[];
    case 9
        sys=[];
    % Unexpected flags %
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
        
end

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

% call simsizes for a sizes structure, fill it in and convert it to a sizes array.
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);

% initialize the initial conditions
x0  = [0;0];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];

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
%u(1)----d(t);u(2)---control input;u(3)-----x2d_dot(derivative of x2d)
function sys=mdlDerivatives(t,x,u,kf,Me,B,Asc,Acog1,Acog3,p)
sys(1)=x(2);
sys(2)=(u(2)-B*x(2)-Asc*sat(kf*x(2))+Acog1*sin(x(1)*2*pi/p)+Acog3*sin(x(1)*6*pi/p)+u(1))/Me;

%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
function sys=mdlUpdate(t,x,u)

sys = [];

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys=mdlOutputs(t,x,u)
sys = x;

%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
function sys=mdlTerminate(t,x,u)

sys = [];

function f=sat(para)
if para>1
    f=1;
elseif para<-1
    f=-1;
else
    f=para;
end
