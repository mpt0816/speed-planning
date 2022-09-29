global order
global wn
global zeta
global tau

if order == 2
    numerator = [wn * wn];
    denominator = [1, 2 * wn * zeta, wn * wn];
else
    numerator = [1];
    denominator = [tau, 1];
end
figure;
DynamicRespond(numerator, denominator)