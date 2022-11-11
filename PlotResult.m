function PlotResult(planning_data)

global thw

%% simulation timestamp
timestamp = planning_data(:,1);
%% station of ego car
station_ego = planning_data(:,2);
%% velocity of ego car
velocity_ego = planning_data(:,3);
%% acceleration of ego car
acceleration_ego = planning_data(:,4);
%% jerk of ego car
jerk_ego = planning_data(:,5);
%% station of ahead obstacle car
station_obstacle = planning_data(:,6);
%% velocity of ahead obstacle car
velocity_obstacle = planning_data(:,7);
%% acceleration of ahead obstacle car
acceleration_obstacle = planning_data(:,8);

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

%% performance
figure;
actual_follow_distance = planning_data(:,6) - planning_data(:,2); 

track_reserved_time = 1.1 + 1 * (thw - 0);
track_reserved_distance = 23.0 + max(0.0, thw * 1);
command_follow_distance = track_reserved_time * planning_data(:,3) + track_reserved_distance;

plot(timestamp, actual_follow_distance, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp, command_follow_distance, '-r', 'LineWidth', 0.5);

title('Follow Distance');
xlabel('time(s)');
ylabel('station(m)');
legend('actual follow distance', 'required follow distance');
grid on;




