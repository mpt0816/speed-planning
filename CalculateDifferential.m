function value = CalculateDifferential(A, B, x, u)
value = A * x + B * u;

global upper_acceleration
global lower_acceleration
global upper_jerk
global lower_jerk
global upper_diff_jerk
global lower_diff_jerk

value(1, 1) = min(max(value(1, 1), lower_acceleration), upper_acceleration);
value(2, 1) = min(max(value(2, 1), lower_jerk), upper_jerk);
value(3, 1) = min(max(value(3, 1), lower_diff_jerk), upper_diff_jerk);
end

