function sys = CalculateObstacleInfo(ego_speed)
%% params
global obstacle_distance_init
global obstacle_velocity_init
global obstacle_accelaration_init
global sample_time

distance_init = obstacle_distance_init;
obstalce_speed_init = obstacle_velocity_init;
obstacle_acc = obstacle_accelaration_init;

persistent obstalce_speed;
if isempty(obstalce_speed)
    obstalce_speed = obstalce_speed_init;
end

persistent distance;
if isempty(distance)
    distance = distance_init;
end

distance = distance + (obstalce_speed - ego_speed) * sample_time;
obstalce_speed = max(0.0, obstalce_speed + obstacle_acc * sample_time);

sys.s = distance;
sys.v = obstalce_speed;
