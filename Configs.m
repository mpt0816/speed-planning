%% Vehicle model
global order
global wn
global zeta
global tau

%% states limits
global upper_velocity
global lower_velocity
global upper_acceleration
global lower_acceleration
global upper_jerk
global lower_jerk
global upper_diff_jerk
global lower_diff_jerk

%% deceleration limits
global max_dec_sch_speed_0
global max_dec_sch_thresh_0
global max_dec_sch_speed_1
global max_dec_sch_thresh_1

global com_dec_sch_speed_0
global com_dec_sch_thresh_0
global com_dec_sch_speed_1
global com_dec_sch_thresh_1

global jerk_sch_speed_0
global jerk_sch_thresh_0
global jerk_sch_speed_1
global jerk_sch_thresh_1

%% PID
global gain_distance
global gain_speed

%% sample
global time_span
global sample_time

order = 2;
wn = 5.0;
zeta = 0.85;
tau = 0.4;

upper_velocity = 150.0 / 3.6;
lower_velocity = 0.0;
upper_acceleration = 3.0;
lower_acceleration = -5.0;
upper_jerk = 1.0;
lower_jerk = -1.0;
upper_diff_jerk = 5.0;
lower_diff_jerk = -5.0;


max_dec_sch_speed_0 = 5.0;
max_dec_sch_thresh_0 = -3.5;
max_dec_sch_speed_1 = 15.0;
max_dec_sch_thresh_1 = -2.0;

com_dec_sch_speed_0 = 5.0;
com_dec_sch_thresh_0 = -2.0;
com_dec_sch_speed_1 = 15.0;
com_dec_sch_thresh_1 = -1.0;

jerk_sch_speed_0 = 5.0;
jerk_sch_thresh_0 = -2.9;
jerk_sch_speed_1 = 20.0;
jerk_sch_thresh_1 = -2.4;

gain_distance = 0.2;
gain_speed = 0.8;

time_span = 8.0;
sample_time = 0.03;