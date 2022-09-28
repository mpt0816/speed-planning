clc; close all;
%% load data
load('planning_data.mat');
%% simulation timestamp
timestamp = planning_data.Time;
%% station of ego car
station_ego = planning_data.data(:,1);
%% velocity of ego car
velocity_ego = planning_data.data(:,2);
%% acceleration of ego car
acceleration_ego = planning_data.data(:,3);
%% station of ahead obstacle car
station_obstacle = planning_data.data(:,1) + planning_data.data(:,5);
%% velocity of ahead obstacle car
velocity_obstacle = planning_data.data(:,4);
%% acceleration of ahead obstacle car
acceleration_obstacle = planning_data.data(:,6);

data_size = length(acceleration_obstacle);
for i = 1 : 1 : data_size
    if velocity_obstacle(i) < 1e-8
       acceleration_obstacle(i) = 0.0;
    end
end

jerk_ego = [];
for i = 1 : 1 : data_size
    if i == 1
        jerk_ego = [jerk_ego, 0.0];
    else
        jerk = (acceleration_ego(i) - acceleration_ego(i - 1)) / 0.002;
        jerk_ego = [jerk_ego, jerk];
    end
end

figure;
%% plot simulation result
subplot(4, 1, 1);
plot(timestamp, station_ego, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp, station_obstacle, '--b', 'LineWidth', 0.5);

title('station');
xlabel('time(s)');
ylabel('station(m)');
legend('ego car', 'ahead car');


subplot(4, 1, 2);
plot(timestamp, velocity_ego, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp, velocity_obstacle, '--b', 'LineWidth', 0.5);

title('velocity');
xlabel('time(s)');
ylabel('velocity(m/s)');
legend('ego car', 'ahead car');

subplot(4, 1, 3);
plot(timestamp, acceleration_ego, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp, acceleration_obstacle, '--b', 'LineWidth', 0.5);

title('acceleration');
xlabel('time(s)');
ylabel('acceleration(m/s2)');
legend('ego car', 'ahead car');

subplot(4, 1, 4);
plot(timestamp, jerk_ego, '-b', 'LineWidth', 0.5);

title('jerk');
xlabel('time(s)');
ylabel('jerk(m/s3)');
legend('ego car');


