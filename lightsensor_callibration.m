V_vis = []';
V_ir = []';
depth = []'; %Y series data
V_ratio = V_ir./V_vis; %X series data

% %Code Test 1 - Uncomment to verify working code
% x = 0.1:.1:3.3;
% y = 2.376*log(0.52.*x) +0.2*randn(size(x));
% V_ratio = x';
% depth = y';

% %Code Test 2 - Uncomment to verify working code
% x = 1.5:.1:3.3;
% y = 0.73*log(1.1.*x) +0.1*randn(size(x));
% V_ratio = x';
% depth = y';

% %Code Test 3 - Uncomment to verify working code
% x = 0.1:.1:3.3;
% y = -0.5*log(2.1.*x) + 0.2*randn(size(x));
% V_ratio = x';
% depth = y';

%Create a fit for the data in the form Z(V_rat) = A*ln(B*V_rat)
ft = fittype('A*log(B*x)');
[model, gof] = fit(V_ratio, depth, ft);

figure(1)
plot(model, x, y)
title("Depth vs Voltage Ratio of Light Sensors")
xlabel("Voltage V (teensy units)")
ylabel("Depth Z (cm)")

%display model parameters and GOF information
model
gof







