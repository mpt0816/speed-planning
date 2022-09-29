clc; clear all; close all;
InitCondition
Configs

global time_span
global sample_time
global obstacle_distance_init
global obstacle_velocity_init

num_of_pts = floor(time_span / sample_time);
planning_data = zeros(num_of_pts, 6);

command = 0.0;
obs.s = obstacle_distance_init;
obs.v = obstacle_velocity_init;

for i = 1 : num_of_pts
    timestamp = (i - 1)  * sample_time;
    states = VehicleDynamicRespond(command);
    
    planning_data(i, : ) = [timestamp, states.s, states.v, states.a, obs.s, obs.v];
    
    obs = CalculateObstacleInfo(states.v);
    command =  CalculateAccelaration(states.v, obs.s, obs.v);
end

PlotResult(planning_data);
% PlotVehicleDynamicRespond;