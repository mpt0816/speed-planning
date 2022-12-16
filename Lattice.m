function data = Lattice()

%% ego
global is_obstacle_ahead
global obstacle_distance_init
global obstacle_velocity_init
global obstacle_accelaration_init

%% obstacle
global ego_satation_init
global ego_velocity_init
global ego_accelaration_init
global ego_jerk_init

%% target
global curise_velocity
global thw
global time_span

%% states limits
global upper_velocity
global lower_velocity
global upper_acceleration
global lower_acceleration
global upper_jerk
global lower_jerk
global upper_diff_jerk
global lower_diff_jerk


ego.s = ego_satation_init;
ego.v = ego_velocity_init;
ego.a = ego_accelaration_init;
ego.j = ego_jerk_init;

obs.is_obstacle_ahead = is_obstacle_ahead;
obs.s = obstacle_distance_init;
obs.v = obstacle_velocity_init;
obs.a = obstacle_accelaration_init;

target.v = curise_velocity;
target.thw = thw;
target.time_span = time_span;
target.time_interval = 0.2;

constrant.upper_v = upper_velocity;
constrant.lower_v = lower_velocity;
constrant.upper_a = upper_acceleration;
constrant.lower_a = lower_acceleration;
constrant.upper_j = upper_jerk;
constrant.lower_j = lower_jerk;
constrant.upper_dj = upper_diff_jerk;
constrant.lower_dj = lower_diff_jerk;

weights.thw = 100.0;
weights.v = 100.0;
weights.jerk = 10.0;

cruise_sample = CuriseEndPointSample(target, obs);
follow_sample = FollowObstacleEndPointSample(target, obs);
end_samples = [cruise_sample; follow_sample];

ploys = [];
min_cost = 1e10;
for ii = [1 : 1 : length(end_samples)]
    ploy = CalculatePloy(ego, end_samples(ii));
    if ~KinematicConstraints(ploy)
        continue;
    end
    ploy.cost = PloyCost(ploy);
    if ploy.cost < min_cost
        min_cost = ploy.cost;
        best_ploy = ploy;
    end
    ploys = [ploys; ploy];
end
data = DataTransform(best_ploy, target, obs);

PlotPloys(ploys, best_ploy, obs, target);


    function cost = PloyCost(ploy)
        cost_thw = 0;
        cost_jerk = 0;
        cost_v = 0;
        
        ref_v = target.v;
        if obs.is_obstacle_ahead
            ref_v = obs.v;
        end
        
        track_reserved_time = 1.1 + 1 * (target.thw - 0);
        track_reserved_distance = 23.0 + max(0.0, target.thw * 1);
        t = target.time_interval;
        while t <= target.time_span
            [obs_s, obs_v, obs_a]= CalculateObstalceSVA(obs, t);
            s = PloyEval(ploy, 0, t);
            v = PloyEval(ploy, 1, t);
            a = PloyEval(ploy, 2, t);
            j = PloyEval(ploy, 3, t);
            
            thw_dis = track_reserved_distance + track_reserved_time * v;
            if obs_s - s >= thw_dis
                cost_thw = cost_thw + 0;
            else
                %% 归一化到[0, 1]
                cost_thw = cost_thw + (1 - (obs_s - s) / thw_dis);
            end
            
            if j > 0
                cost_jerk = cost_jerk + abs(j / upper_diff_jerk);
            else
                cost_jerk = cost_jerk + abs(j / lower_diff_jerk);
            end
            if obs.is_obstacle_ahead
                [s, ref_v, a] = CalculateObstalceSVA(obs, t);
                end_pt.v = v;
            end
            cost_v = cost_v + abs(ref_v - v) / upper_velocity;
            
            t = t + target.time_interval;
        end
        
        num_of_knots = floor(target.time_span / target.time_interval);
        cost_thw = cost_thw / num_of_knots;
        cost_jerk = cost_jerk / num_of_knots;
        cost_v = cost_v / num_of_knots;
        
        cost = weights.thw * cost_thw + weights.jerk * cost_jerk + weights.v * cost_v;
    end

    function out = KinematicConstraints(ploy)
        t = 0;
        while t <= target.time_span
            [obs_s, obs_v, obs_a]= CalculateObstalceSVA(obs, t);
            s = PloyEval(ploy, 0, t);
            if s >= obs_s
                out = false;
                return;
            end
            v = PloyEval(ploy, 1, t);
            if v > upper_velocity || v < lower_velocity
                out = false;
                return;
            end
            a = PloyEval(ploy, 2, t);
            if a > upper_acceleration || a < lower_acceleration
                out = false;
                return;
            end
            j = PloyEval(ploy, 3, t);
            if j > upper_jerk || j < lower_jerk
                out = false;
                return;
            end
            t = t + target.time_interval;
        end
        out = true;
    end

    function ploy = CalculatePloy(init_pt, end_pt)
        if end_pt.label == "quartic"
            ploy.coef = QuarticPloy(init_pt, end_pt);
            ploy.t = end_pt.t;
            ploy.label = "quartic";
            return;
        end
        
        if end_pt.label == "quintic"
            ploy.coef = QuinticPloy(init_pt, end_pt);
            ploy.t = end_pt.t;
            ploy.label = "quintic";
            return;
        end
    end

    function value = PloyEval(ploy, order, t)
        coef = ploy.coef;
        type = length(coef);
        value = 0;
        duration = t;
        if t > ploy.t
            t = ploy.t;
        end
        if order == 0
            for  i = [type : -1 : 1]
                value = value * t;
                value = value + coef(i);
            end
            if duration > ploy.t
                v = 0;
                for i = [type : -1 : 2]
                    v = v * t;
                    v = v + (i - 1) * coef(i);
                end
                value = value + (duration - ploy.t) * v;
            end
        elseif order == 1
                for i = [type : -1 : 2]
                    value = value * t;
                    value = value + (i - 1) * coef(i);
                end
        elseif order == 2
                for i = [type : -1 : 3]
                    value = value * t;
                    value = value + (i - 1) * (i - 2) * coef(i);
                end
                
        elseif order == 3
                for i = [type : -1 : 4]
                    value = value * t;
                    value = value + (i - 1) * (i - 2) * (i - 3) * coef(i);
                end 
                if t >=  ploy.t
                    value = 0;
                end
        end   
    end

    function coef = QuarticPloy(init_pt, end_pt)
        t = end_pt.t;
        t2 = t * t;
        t3 = t2 * t;
        
        re = init_pt.s;
        rd = init_pt.v;
        rc = init_pt.a * 0.5;
        rb = (3 * (end_pt.v - init_pt.v) - 4 * rc * t - end_pt.a * t) / (3 * t2);
        ra = (init_pt.v - end_pt.v + rc * t + 0.5 * end_pt.a * t) / (2 * t3);
        coef = [re, rd, rc, rb, ra];
    end


    function coef = QuinticPloy(init_pt, end_pt)
        t = end_pt.t;
        t2 = t * t;
        t3 = t2 * t;
        t4 = t3 * t;
        t5 = t4 * t;
        
        rf = init_pt.s;
        re = init_pt.v;
        rd = init_pt.a * 0.5;
        rc = (20 * (end_pt.s - init_pt.s) - (8 * end_pt.v + 12 * init_pt.v) * t + (end_pt.a - 3 * init_pt.a) * t2) / 2 / t3;
        rb = (30 * (init_pt.s - end_pt.s) + (14 * end_pt.v + 16 * init_pt.v) * t + (3 * init_pt.a - 2 * end_pt.a) * t2) / 2 / t4;
        ra = (12 * (end_pt.s - init_pt.s) - 6 * (end_pt.v + init_pt.v) * t + (end_pt.a - init_pt.a) * t2) / 2 / t5;
        coef = [rf, re, rd, rc, rb, ra];
    end

    function end_points = CuriseEndPointSample(target, obs)
        end_points = [];
        ref_v = target.v;

        t = target.time_interval;
        end_pt.s = 1e10;
        end_pt.v = ref_v;
        end_pt.a = 0.0;
        end_pt.label = "quartic";
        while t <= 2 * target.time_span
            for v = 0 : 5 / 3.6 : target.v + 5 / 3.6
                end_pt.t = t;
                end_pt.v = v;
                if ~FeasibleRegion(end_pt)
                    continue;
                end
                end_points = [end_points; end_pt]; 
            end
            t = t + target.time_interval;   
        end
    end

    function end_points = FollowObstacleEndPointSample(target, obs)
        end_points = [];
        
        if ~obs.is_obstacle_ahead
            return;
        end
        
        t = target.time_interval;
        while t <= target.time_span
            [s, v, a] = CalculateObstalceSVA(obs, t);
            s_sample = FollowBounds(s, v, target.thw);
            num = length(s_sample);
            for i = [1 : 1 : num]
                end_pt.s = s_sample(i);
                end_pt.v = v;
                end_pt.a = 0;
                end_pt.t = t;
                end_pt.label = "quintic";
                if ~FeasibleRegion(end_pt)
                    continue;
                end
                end_points = [end_points; end_pt];
            end
            t = t + target.time_interval;
        end
    end

    function s_sample = FollowBounds(s, v, thw)
        s_sample = [];
        track_reserved_time = 1.1 + 1 * (thw - 0);
        track_reserved_distance = 23.0 + max(0.0, thw * 1);
        s = s - 5;
        while s > 0.0
            s_sample = [s_sample; s];
            s = s - 5;
        end
    end

    function [s, v, a] = CalculateObstalceSVA(obs, time)
            break_time = 1e10;
            if obs.a < 0.0
                break_time = abs(obs.v / obs.a);
            end
            
            if time >= break_time
                s = obs.s + abs(obs.v * obs.v / 2 / obs.a);
                v = 0.0;
                a = 0.0;
            else
                s = obs.s + obs.v * time + 0.5 * obs.a * time * time;
                v = obs.v + obs.a * time;
                a = obs.a;
            end
            
            if ~obs.is_obstacle_ahead
                s = 1e10;
            end
            
%             if time > 3
%                 s = obs.s + obs.v * 3;
%                 v = 0.0;
%                 a = 0.0;
%             end
    end

    function out = FeasibleRegion(end_pt)
        t = end_pt.t;
        v_lower = max(lower_velocity, ego_velocity_init + lower_acceleration * t);
        v_upper = min(upper_velocity, ego_velocity_init + upper_acceleration * t);
        s_lower = ego_satation_init + (ego_velocity_init * ego_velocity_init - v_lower * v_lower) / (2 * abs(lower_acceleration));
        s_upper = ego_satation_init + (v_upper * v_upper - ego_velocity_init * ego_velocity_init) / (2 * abs(upper_acceleration));
        if end_pt.s > 1e5
            out = end_pt.v >= v_lower && end_pt.v <= v_upper;
        else
            out = (end_pt.s >= s_lower) && (end_pt.s <= s_upper) && (end_pt.v >= v_lower) && (end_pt.v <= v_upper);
        end
    end

    function planning_data = DataTransform(ploy, target, obs)
        num_of_knots = floor(target.time_span / target.time_interval);
        planning_data = zeros(num_of_knots, 8);
        for i = 0 : num_of_knots - 1
            timestamp = i * target.time_interval;
            s = PloyEval(ploy, 0, timestamp);
            v = PloyEval(ploy, 1, timestamp);
            a = PloyEval(ploy, 2, timestamp);
            j = PloyEval(ploy, 3, timestamp);
            [obs_s, obs_v, obs_a] = CalculateObstalceSVA(obs, timestamp);
            planning_data(i + 1, : ) = [timestamp, s, v, a, j, obs_s, obs_v, obs_a];
        end
    end

    function PlotPloys(ploys, best_ploy, obs, target)
        num_of_knots = floor(target.time_span / target.time_interval);
        obs_data = zeros(num_of_knots, 5);
        for i = 0 : num_of_knots - 1
            timestamp = i * target.time_interval;
            [obs_s, obs_v, obs_a] = CalculateObstalceSVA(obs, timestamp);
            obs_data(i + 1, : ) = [timestamp, obs_s, obs_v, obs_a, 0];
        end
        
        figure;
        subplot(4, 1, 1);
        plot(obs_data(:, 1), obs_data(:, 2), '-r', 'LineWidth', 1.0);
        
        ploy_data = zeros(num_of_knots, 2);
        for j = [1 : 1 : length(ploys)]
            for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                s = PloyEval(ploys(j), 0, timestamp);
                ploy_data(i + 1, : ) = [timestamp, s];
            end
            hold on;
            if ploys(j).label == "quartic"
                plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 0.5);
            else
                plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 0.5);
            end
        end
        
        for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                s = PloyEval(best_ploy, 0, timestamp);
                ploy_data(i + 1, : ) = [timestamp, s];
        end
        hold on;
        if best_ploy.label == "quartic"
            plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 2.0);
        else
            plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 2.0);
        end

        
        
        subplot(4, 1, 2);
        plot(obs_data(:, 1), obs_data(:, 3), '-r', 'LineWidth', 1.0);
       
        for j = [1 : 1 : length(ploys)]
            for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                v = PloyEval(ploys(j), 1, timestamp);
                ploy_data(i + 1, : ) = [timestamp, v];
            end
            hold on;
            if ploys(j).label == "quartic"
                plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 0.5);
            else
                plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 0.5);
            end
        end
        
        for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                s = PloyEval(best_ploy, 1, timestamp);
                ploy_data(i + 1, : ) = [timestamp, s];
        end
        hold on;
        if best_ploy.label == "quartic"
            plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 2.0);
        else
            plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 2.0);
        end
        
        subplot(4, 1, 3);
        plot(obs_data(:, 1), obs_data(:, 4), '-r', 'LineWidth', 1.0);
       
        for j = [1 : 1 : length(ploys)]
            for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                a = PloyEval(ploys(j), 2, timestamp);
                ploy_data(i + 1, : ) = [timestamp, a];
            end
            hold on;
            if ploys(j).label == "quartic"
                plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 0.5);
            else
                plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 0.5);
            end
        end
        
        for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                s = PloyEval(best_ploy, 2, timestamp);
                ploy_data(i + 1, : ) = [timestamp, s];
        end
        hold on;
        if best_ploy.label == "quartic"
            plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 2.0);
        else
            plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 2.0);
        end
        
        subplot(4, 1, 4);
        plot(obs_data(:, 1), obs_data(:, 5), '-r', 'LineWidth', 1.0);
        
        for j = [1 : 1 : length(ploys)]
            for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                jerk = PloyEval(ploys(j), 3, timestamp);
                ploy_data(i + 1, : ) = [timestamp, jerk];
            end
            hold on;
            if ploys(j).label == "quartic"
                plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 0.5);
            else
                plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 0.5);
            end
        end
        
        for i = 0 : num_of_knots - 1
                timestamp = i * target.time_interval;
                s = PloyEval(best_ploy, 3, timestamp);
                ploy_data(i + 1, : ) = [timestamp, s];
        end
        hold on;
        if best_ploy.label == "quartic"
            plot(ploy_data(:, 1), ploy_data(:, 2), '--', 'LineWidth', 2.0);
        else
            plot(ploy_data(:, 1), ploy_data(:, 2), '-', 'LineWidth', 2.0);
        end
          
    end

end