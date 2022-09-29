function value = OED_RK4(func, x, u, step)
sys = VehicleDynamicSystem();
s1 = func(sys.A, sys.B, x, u);
s2 = func(sys.A, sys.B, x + s1 * step / 2.0, u);
s3 = func(sys.A, sys.B, x + s2 * step / 2.0, u);
s4 = func(sys.A, sys.B, x + s3 * step, u);
value = x + step / 6.0 * (s1 + 2 * s2 + 2 * s3 + s4);
end

