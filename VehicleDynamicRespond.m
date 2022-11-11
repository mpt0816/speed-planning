function sys = VehicleDynamicRespond(command)

%% state variable
global ego_satation_init
global ego_velocity_init
global ego_accelaration_init
global ego_jerk_init
global sample_time

persistent init
persistent s
persistent v
persistent a
persistent jerk

if isempty(init)
    init = false;
end

if isempty(s)
    s = ego_satation_init;
end

if isempty(v)
    v = ego_velocity_init;
end

if isempty(a)
    a = ego_accelaration_init;
end

if isempty(jerk)
    jerk = ego_jerk_init;
end

if ~init
    sys.s = s;
    sys.v = v;
    sys.a = a;
    sys.jerk = jerk;
    init = true;
    return;
end

%% system model
states = [v; a; jerk];
input = [command];

f = @CalculateDifferential;
states = OED_RK4(f, states, input, sample_time);

global upper_velocity
global lower_velocity
global upper_acceleration
global lower_acceleration
global upper_jerk
global lower_jerk

v = min(max(states(1, 1), lower_velocity), upper_velocity);
a = min(max(states(2, 1), lower_acceleration), upper_acceleration);
jerk = min(max(states(3, 1), lower_jerk), upper_jerk);
s = s + states(1, 1) * sample_time;
sys.s = s;
sys.v = v;
sys.a = a;
sys.jerk = jerk;
