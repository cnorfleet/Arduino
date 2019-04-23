%Just in case our data looks more linear than logarithmic over our choosen
%data range


V_vis = [];
V_ir = [];
depth = [];
V_ratio = V_ir./V_vis;

%Test 1 - uncommnent to verify code is working
% V_ratio = 0:0.1:3.3;
% depth = 2.34.*V_ratio -0.13 + randn(size(V_ratio));



fit = polyfit(V_ratio, depth, 1);

best_fit = fit(1).*V_ratio + fit(2);

figure(1)
plot(V_ratio, best_fit, "k")
hold on;
plot(V_ratio, depth, "rx")
title("Linear Fit for Depth vs Voltage Ratio of Light Sensors")
xlabel("Voltage Ratio V (teensy units)")
ylabel("Depth Z (cm")
hold off;


fit
