function data = QP()

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

weights.v = 1000.0;
weights.a = 1000.0;
weights.j = 1000.0;
weights.relaxation_factor = 100;

problem = FormulateQPProblem(ego, obs, constrant, target, weights);
qp_results = SolveQPProblem(problem);

if qp_results.info.status_val < 0 || (~(qp_results.info.status_val == 1) && ~(qp_results.info.status_val == 2))
    disp('Osqp Failed');
    return;
end

data = DataTransform(qp_results, target, obs);

    function problem = FormulateQPProblem(ego, obs, cons, target, weights)
        kernel = CalcaulateKernel(target, weights, obs);
        problem.H = kernel.H;
        problem.G = kernel.G;
        constraint = CalculateConstraint(target, cons, ego, obs);
        problem.A = constraint.A;
        problem.lower = constraint.lower;
        problem.upper = constraint.upper;
    end
   
    function results = SolveQPProblem(problem)
            P = sparse(problem.H);
            q = problem.G;
            A = sparse(problem.A);
            l = problem.lower;
            u = problem.upper;
            
            solver = osqp;
            settings = OsqpSettings(solver);
            solver.setup(P, q, A, l, u, settings);
            results = solver.solve();     
    end

    function settings = OsqpSettings(solver)
            settings = solver.default_settings();
            settings.max_iter = 5000;
            settings.polish = true;
            settings.verbose = false;
            settings.scaled_termination = false;
            settings.warm_start = false;
    end

    function kernel = CalcaulateKernel(target, weights, obs)
            num_of_knots = floor(target.time_span / target.time_interval);
            kernel_dim = 5 * num_of_knots;
            kernel.H = zeros(kernel_dim, kernel_dim);
            kernel.G = zeros(kernel_dim, 1);
            ref_v = target.v;
            if obs.is_obstacle_ahead
                ref_v = obs.v;
            end
            
            for i = 0 : 1 : num_of_knots - 1
                kernel.H(5 * i + 2, 5 * i + 2) = weights.v;
                kernel.H(5 * i + 3, 5 * i + 3) = weights.a;
                kernel.H(5 * i + 4, 5 * i + 4) = weights.j;
                kernel.H(5 * i + 5, 5 * i + 5) = weights.relaxation_factor;
                kernel.G(5 * i + 2, 1) = - weights.v * ref_v;
            end
    end

    function constriant = CalculateConstraint(target, cons, ego, obs)
        dt = target.time_interval;
        dt2 = dt * dt;
        dt3 = dt * dt2;
        num_of_knots = floor(target.time_span / target.time_interval);
        kernel_dim = 5 * num_of_knots;
        num_of_constriant = 11 * num_of_knots - 1;
        A = zeros(num_of_constriant, kernel_dim);
        lower = zeros(num_of_constriant, 1);
        upper = zeros(num_of_constriant, 1);
        
        rows = 1;
        %% 起点约束, 4个
        A(rows : rows + 3, 1 : 4) = eye(4);
        lower(rows : rows + 3, 1) = [ego.s; ego.v; ego.a; ego.j];
        upper(rows : rows + 3, 1) = [ego.s; ego.v; ego.a; ego.j];
        rows = rows + 4;
        
        %% 运动学约束, 3(n-1)个
        for i = 0 : 1 : num_of_knots - 2
            %% acc, jerk
            A(rows, 5 * i + 3 : 5 * i + 4) = [1, dt];
            A(rows, 5 * (i + 1) + 3) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 0;
            rows = rows + 1;
            
            %% v, acc, jerk
            A(rows, 5 * i + 2 : 5 * i + 4) = [1, dt, 0.5 * dt2];
            A(rows, 5 * (i + 1) + 2) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 0;
            rows = rows + 1;
            
            %% s, v, acc, jerk
            A(rows, 5 * i + 1 : 5 * i + 4) = [1, dt, 0.5 * dt2, 1/6 * dt3];
            A(rows, 5 * (i + 1) + 1) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 0;
            rows = rows + 1;
        end
        
        %% 只能前进，不能倒车, n-1个
        for i = 0 : 1 : num_of_knots - 2
            A(rows, 5 * i + 1) = -1;
            A(rows, 5 * (i + 1) + 1) = 1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 1e10;
            rows = rows + 1;
        end
        
        %% 边界约束, 3n个
        for i = 0 : 1 : num_of_knots - 1
            A(rows : rows + 2, 5 * i + 2 : 5 * i + 4) = eye(3);
            lower(rows : rows + 2, 1) = [cons.lower_v; cons.lower_a; cons.lower_j];
            upper(rows : rows + 2, 1) = [cons.upper_v; cons.upper_a; cons.upper_j];
            rows = rows + 3;
        end
        
        %% djerk 约束, n-1个
        for i = 0 : 1 : num_of_knots - 2
            A(rows, 5 * i + 4) = -1;
            A(rows, 5 * (i + 1) + 4) = 1;
            lower(rows, 1) = cons.lower_dj;
            upper(rows, 1) = cons.upper_dj;
            rows = rows + 1;
        end
        
        %% Twh约束, n个
        %% 碰撞约束, n个
        track_reserved_time = 1.1 + 1 * (target.thw - 0);
        track_reserved_distance = 23.0 + max(0.0, target.thw * 1);
        for i = 0 : 1 : num_of_knots - 1
            time = dt * i;
            [obs_s, obs_v, obs_a] = CalculateObstalceSVA(obs, time);
            A(rows, 5 * i + 1 : 5 * i + 2) = [1, track_reserved_time];
            A(rows, 5 * i + 5) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = obs_s - track_reserved_distance;
            rows = rows + 1;
            
            A(rows, 5 * i + 1) = 1;
            lower(rows, 1) = 0;
            upper(rows, 1) = obs_s;
            rows = rows + 1;    
        end
        
        %% 松弛因子约束, n个
        for i = 0 : 1 : num_of_knots - 1
            A(rows, 5 * i + 5) = 1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 100;
            rows = rows + 1;
        end
        
        
        constriant.A = A;
        constriant.lower = lower;
        constriant.upper = upper;
    end

    function planning_data = DataTransform(qp_results, target, obs)
        num_of_knots = floor(target.time_span / target.time_interval);
        planning_data = zeros(num_of_knots, 8);
        for i = 0 : num_of_knots - 1
            timestamp = i * target.time_interval;
%             disp(qp_results.x(5 * i + 5))
            [obs_s, obs_v, obs_a] = CalculateObstalceSVA(obs, timestamp);
            planning_data(i + 1, : ) = [timestamp, qp_results.x(5 * i + 1), qp_results.x(5 * i + 2), qp_results.x(5 * i + 3), qp_results.x(5 * i + 4), obs_s, obs_v, obs_a];
        end
    end

    function [s, v, a] = CalculateObstalceSVA(obs, time)
            braek_time = 1e10;
            if obs.a < 0.0
                break_time = fabs(obs.v / obs.a);
            end
            
            if time >= braek_time
                s = obs.s + fabs(obs.v * obs.v / 2 / obs.a);
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
    end

end