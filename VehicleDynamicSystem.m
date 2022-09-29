function y = VehicleDynamicSystem()
global order
global wn
global zeta
global tau

if order == 2
    A = [0, 1, 0; 0, 0, 1; 0, -wn * wn, -2 * wn * zeta];
    B = [0; 0; wn * wn];
else
    A = [0, 1, 0; 0, -1.0 / tau, 1; 0, 0, 0];
    B = [0; 1.0 / tau; 0];
end

y.A = A;
y.B = B;
end

