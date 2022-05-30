function [sys,x0,str,ts,simStateCompliance] = CalculateObstacleInfo(t,x,u,flag)
switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1
    sys=mdlDerivatives(t,x,u);

  case 2
    sys=mdlUpdate(t,x,u);

  case 3
    sys=mdlOutputs(t,x,u);

  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
%% output: 
% obstalce init distance
% obstalce init speed
% obstalce init acc
% ego speed
sizes.NumInputs     = 5;
%% input:
% ahead obstacle velocity
% distance
sizes.NumOutputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1; 

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u)

sys = [];

function sys=mdlOutputs(t,x,u)
%% params

distance_init = u(1);
obstalce_speed_init = u(2);
obstacle_acc = u(3);
ego_speed = u(4);
timestamp = u(5);

persistent lsat_timestamp;
if isempty(lsat_timestamp)
    lsat_timestamp = 0.0;
end

persistent is_init;
if isempty(is_init)
    is_init = false;
end

persistent obstalce_speed;
if isempty(obstalce_speed)
    obstalce_speed = obstalce_speed_init;
end

persistent distance;
if isempty(distance)
    distance = distance_init;
end

sample_time = timestamp - lsat_timestamp;

if is_init == true
    distance = distance + (obstalce_speed - ego_speed) * sample_time;
    obstalce_speed = max(0.0, obstalce_speed + obstacle_acc * sample_time);
end
is_init = true;
lsat_timestamp = timestamp;
sys = [obstalce_speed, distance];

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];
