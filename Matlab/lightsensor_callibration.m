V_ir = 25:50:1025;
depth = -0.56*log(0.25*V_ir) + 0.2*randn(size(V_ir));
%V_vis = []; %uncomment this if we decide we want to use the ratio again

%create x,y variables for both the log and an exp models
x_log = V_ir;
y_log = depth;

x_exp = depth;
y_exp = V_ir;

% %Uncomment this section if you are getting an error with columns
% x_log = x_log';
% y_log = y_log';
% 
% x_exp = x_exp';
% y_exp = y_exp';

%uncomment this if we decide we want to use the ratio again
% x_log = V_ir/V_vis;
% y_log = V_ir/V_vis;

%Create a fit for the data in the form Z(V) = A*ln(B*V)
% ft = fittype('A*log(B*x)');
% [log_model, log_gof] = fit(x_log, y_log, ft, "StartPoint", [-1, 1]);

%Create a fit for the data in the form V(z) = A*exp(B*z)
[exp_model, exp_gof] = fit(x_exp, y_exp, "exp1");

coeffs = coeffvalues(exp_model);

C = 1/coeffs(2);
D = 1/coeffs(1);

x_model = 100:25:1025;
log_model = C*log(D*x_model);

%Plot our two models out
figure(1)
plot(x_log, y_log, "bx")
hold on;
plot(x_model, log_model, "k")
hold off;
title("Logarithmic Model for Depth versus IR Intensity")
xlabel("Ir Sensor Voltage V (Teensy Units)")
ylabel("Depth Z (cm)")


figure(2)
plot(x_exp, y_exp, "bx")
hold on;
plot(exp_model, "k")
hold off;
title("Exponential Model for IR Intensity versus Depth")
ylabel("Ir Sensor Voltage V (Teensy Units)")
xlabel("Depth Z (cm)")

%Output the fit parameters

% log_model

exp_model








