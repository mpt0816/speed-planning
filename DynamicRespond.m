function DynamicRespond(numerator, denominator)

sys = tf(numerator, denominator);
% sys = feedback(sys, 1);

time_range = 8.0;
time_step = 0.01;  
time = 0 : time_step : time_range;
y = step(sys,time);

%% 求最大值，即超调量
[PeakValue, PeakTime] = max(step(sys,time)); 
%% 最大超调量时间
PeakTime = PeakTime * time_step;
%% 求最终响应值,最后10个数平均
ys = mean(y(length(time) - 10 : length(time))); 
%% 超调量百分比
sigma = (PeakValue - ys) / ys; 
%% 稳态误差
ess = 1 - ys;
%% 寻找到第一次y(k)>0.1ys即停止
for k = 1 : length(time)  
    if y(k) >= ys * 0.1
        T0 = k;
        break;
    end
end

%% 寻找到第一次y(k)>0.9ys即停止
for k = 1 : length(time)   
    if y(k) >= ys * 0.9
        T1 = k;
        break;
    end
end

%% 求上升时间
RiseTime = (T1 - T0) * time_step; 

%% 容许范围+-5%
ppm = 0.05;            
yup = ys * (1 + ppm);
ydown = ys * (1 - ppm);

%% 如果峰值在ys的容许范围内
if PeakValue <= yup   
    for k = 1 : PeakTime / time_step
        if y(k) > ydown
            AdjustTime = k * time_step;
            break;
        end
    end
end

%% 如果峰值超出了ys的容许范围
if PeakValue > yup   
    for k = PeakTime / time_step : length(time)
        if (y(k-1) <= ydown && y(k) >= ydown) || (y(k-1)>= yup && y(k) <= yup)
            if max(y(k : length(time))) <= yup && min(y(k : length(time))) >= ydown
                AdjustTime = k * time_step;
                break;
            end
        end
    end
end

%% plot响应图
step(sys, time);

hold on;
plot([PeakTime, PeakTime], [0, y(PeakTime / time_step)], '--', 'color', 'red');

hold on;
plot([AdjustTime, AdjustTime], [0, y(AdjustTime / time_step)], '--', 'color', 'k');

hold on;
plot([AdjustTime, time_range], [ydown, ydown], '--', 'color', 'k');
hold on;
plot([AdjustTime, time_range], [yup, yup], '--', 'color', 'k');

hold on;
plot([T0 * time_step, T0 * time_step], [0, y(T0)], '--', 'color', 'm');
hold on;
plot([T1 * time_step, T1 * time_step], [0, y(T1)], '--', 'color', 'm');

disp(['超调量 = ', num2str(sigma*100), '%'])
disp(['稳态误差 = ', num2str(ess)])
disp(['上升时间 = ', num2str(RiseTime), 's'])
disp(['峰值时间 = ', num2str(PeakTime), 's'])
disp(['调节时间 = ', num2str(AdjustTime), 's, 容许范围在稳态值的±', num2str(ppm * 100), '%'])
end

