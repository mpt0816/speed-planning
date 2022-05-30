function [sys,x0,str,ts,simStateCompliance] = CalculateAccelaration(t,x,u,flag)
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
% acc command
% is_dangerous
sizes.NumOutputs     = 2;
%% input:
% ego velocity
% ahead obstacle velocity
% distance
% max comfort acc
% max acc
sizes.NumInputs      = 5;
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
ttc = 5.0;
user_define_velocity = 100.0 / 3.6;
max_comfort_acc = abs(u(4));
feasable_lower_acc = u(5);
acc_max = abs(max_comfort_acc);
acc_min = -acc_max;
track_reserved_time = 1.1 + 0.4 * (ttc - 1.0);
track_reserved_distance = 3.0 + max(0.0, ttc * 0.5);

gain_station = 0.2;
gain_speed = 0.8;

vel_ego = u(1);
vel_obstacle_ahead = u(2);
distance = u(3);
%% calculate reference speed

ref_velocity = user_define_velocity;

is_obstacle_ahead = true;
is_dangerous = false;

if is_obstacle_ahead
    ref_distance_ahead = track_reserved_time * vel_ego + track_reserved_distance;
    approach_velocity = max(vel_ego - vel_obstacle_ahead, 0.0);
    comfort_approach_distance = approach_velocity * approach_velocity / (2.0 * max_comfort_acc);
    safe_distance = comfort_approach_distance + approach_velocity * 1.5 + track_reserved_distance;
    ref_distance_ahead = ref_distance_ahead + comfort_approach_distance;
    is_dangerous = distance < safe_distance;
    
    if is_dangerous
        acc_min = feasable_lower_acc;
    end
    
    err_distance_ahead = ref_distance_ahead - distance;
    ref_velocity_ahead = -gain_station * err_distance_ahead;
    ref_velocity = vel_obstacle_ahead + ref_velocity_ahead;
end

ref_velocity = min(ref_velocity, user_define_velocity);
acc_cmd = gain_speed * (ref_velocity - vel_ego);
acc_cmd = min(max(acc_cmd, acc_min), acc_max);

sys = [acc_cmd, is_dangerous];

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];
