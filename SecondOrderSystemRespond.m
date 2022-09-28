function SecondOrderSystemRespond(wn, zeta)
numerator = [wn * wn];
denominator = [1 2 * wn * zeta wn * wn];
DynamicRespond(numerator, denominator);
end

