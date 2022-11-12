function ResultComparison(planning_data_pid, planning_data_qp)

global thw

%% PID
%% simulation timestamp
timestamp_pid = planning_data_pid(:,1);
%% station of ego car
station_ego_pid = planning_data_pid(:,2);
%% velocity of ego car
velocity_ego_pid = planning_data_pid(:,3);
%% acceleration of ego car
acceleration_ego_pid = planning_data_pid(:,4);
%% jerk of ego car
jerk_ego_pid = planning_data_pid(:,5);
%% station of ahead obstacle car
station_obstacle_pid = planning_data_pid(:,6);
%% velocity of ahead obstacle car
velocity_obstacle_pid = planning_data_pid(:,7);
%% acceleration of ahead obstacle car
acceleration_obstacle_pid = planning_data_pid(:,8);


%% QP
%% simulation timestamp
timestamp_qp = planning_data_qp(:,1);
%% station of ego car
station_ego_qp = planning_data_qp(:,2);
%% velocity of ego car
velocity_ego_qp = planning_data_qp(:,3);
%% acceleration of ego car
acceleration_ego_qp = planning_data_qp(:,4);
%% jerk of ego car
jerk_ego_qp = planning_data_qp(:,5);
%% station of ahead obstacle car
station_obstacle_qp = planning_data_qp(:,6);
%% velocity of ahead obstacle car
velocity_obstacle_qp = planning_data_qp(:,7);
%% acceleration of ahead obstacle car
acceleration_obstacle_qp = planning_data_qp(:,8);

figure;
%% plot simulation result
subplot(4, 1, 1);
plot(timestamp_pid, station_obstacle_pid, '--r', 'LineWidth', 0.5);
hold on;
plot(timestamp_pid, station_ego_pid, '-b', 'LineWidth', 0.5);
hold on
plot(timestamp_qp, station_ego_qp, '-k', 'LineWidth', 0.5);
% hold on;
% plot(timestamp_qp, station_obstacle_qp, '--b', 'LineWidth', 0.5);

title('station');
xlabel('time(s)');
ylabel('station(m)');
legend('ahead car', 'pid', 'qp');


subplot(4, 1, 2);
plot(timestamp_pid, velocity_obstacle_pid, '--r', 'LineWidth', 0.5);
hold on;
plot(timestamp_pid, velocity_ego_pid, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp_qp, velocity_ego_qp, '-k', 'LineWidth', 0.5);
% hold on;
% plot(timestamp_qp, velocity_obstacle_qp, '--b', 'LineWidth', 0.5);

title('velocity');
xlabel('time(s)');
ylabel('velocity(m/s)');
legend('ahead car', 'pid', 'qp');

subplot(4, 1, 3);
plot(timestamp_pid, acceleration_obstacle_pid, '--r', 'LineWidth', 0.5);
hold on;
plot(timestamp_pid, acceleration_ego_pid, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp_qp, acceleration_ego_qp, '-k', 'LineWidth', 0.5);
% hold on;
% plot(timestamp_qp, acceleration_obstacle_qp, '--b', 'LineWidth', 0.5);

title('acceleration');
xlabel('time(s)');
ylabel('acceleration(m/s2)');
legend('ahead car', 'pid', 'qp');

subplot(4, 1, 4);
plot(timestamp_pid, jerk_ego_pid, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp_qp, jerk_ego_qp, '-k', 'LineWidth', 0.5);

title('jerk');
xlabel('time(s)');
ylabel('jerk(m/s3)');
legend('pid', 'qp');

%% performance

track_reserved_time = 1.1 + 1 * (thw - 0);
track_reserved_distance = 23.0 + max(0.0, thw * 1);

figure;
actual_follow_distance_pid = planning_data_pid(:,6) - planning_data_pid(:,2); 
command_follow_distance_pid = track_reserved_time * planning_data_pid(:,3) + track_reserved_distance;

actual_follow_distance_qp = planning_data_qp(:,6) - planning_data_qp(:,2); 
command_follow_distance_qp = track_reserved_time * planning_data_qp(:,3) + track_reserved_distance;

plot(timestamp_pid, actual_follow_distance_pid, '-b', 'LineWidth', 0.5);
hold on;
plot(timestamp_pid, command_follow_distance_pid, '--b', 'LineWidth', 0.5);
hold on;
plot(timestamp_qp, actual_follow_distance_qp, '-k', 'LineWidth', 0.5);
hold on;
plot(timestamp_qp, command_follow_distance_qp, '--k', 'LineWidth', 0.5);

title('Follow Distance');
xlabel('time(s)');
ylabel('station(m)');
legend('actual follow distance pid', 'required follow distance pid', 'actual follow distance qp', 'required follow distance qp');
grid on;




