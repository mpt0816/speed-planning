function sys = CalculateAccelaration(vel_ego, distance, vel_obstacle_ahead)
%% params
global curise_velocity
global thw
global lower_jerk
global ego_velocity_init

global max_dec_sch_speed_0
global max_dec_sch_thresh_0
global max_dec_sch_speed_1
global max_dec_sch_thresh_1

global com_dec_sch_speed_0
global com_dec_sch_thresh_0
global com_dec_sch_speed_1
global com_dec_sch_thresh_1

global gain_distance
global gain_speed
global is_obstacle_ahead

ref_velocity = curise_velocity;
% is_obstacle_ahead = true;
is_dangerous = false;

max_comfort_acc = LinearInterpolation(ego_velocity_init, com_dec_sch_speed_0, com_dec_sch_thresh_0, com_dec_sch_speed_1, com_dec_sch_thresh_1);
feasable_lower_acc = LinearInterpolation(ego_velocity_init, max_dec_sch_speed_0, max_dec_sch_thresh_0, max_dec_sch_speed_1, max_dec_sch_thresh_1);
acc_max = abs(max_comfort_acc);
acc_min = -acc_max;

% track_reserved_time = 1.1 + 0.4 * (thw - 1);
% track_reserved_distance = 3.0 + max(0.0, thw * 0.5);

track_reserved_time = 1.1 + 1 * (thw - 0);
track_reserved_distance = 23.0 + max(0.0, thw * 1);

%% calculate reference speed
if is_obstacle_ahead
    acc_min = -5.0;
    ref_distance_ahead = track_reserved_time * vel_ego + track_reserved_distance;
    approach_velocity = max(vel_ego - vel_obstacle_ahead, 0.0);
    comfort_approach_distance = 0;
    safe_distance = comfort_approach_distance + approach_velocity * 1.5 + track_reserved_distance;
    ref_distance_ahead = ref_distance_ahead + comfort_approach_distance;
    is_dangerous = distance < safe_distance;
    if is_dangerous
        acc_min = -5;
        lower_jerk = -3;
    end
    err_distance_ahead = ref_distance_ahead - distance;
    ref_velocity_ahead = -gain_distance * err_distance_ahead;
    ref_velocity = vel_obstacle_ahead + ref_velocity_ahead;
end

ref_velocity = min(ref_velocity, curise_velocity);
acc_cmd = gain_speed * (ref_velocity - vel_ego);
acc_cmd = min(max(acc_cmd, acc_min), acc_max);

sys = acc_cmd;
