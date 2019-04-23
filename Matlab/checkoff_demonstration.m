turbp = A00;
turb90 = A01;
vis = A02;
ir = A03;
pressure = A15;
led = A16;

figure(1)
title("Turbidity Sensor Outputs")

subplot(2,1,1)
plot(turbp);
title("Passing Light Readout")
ylabel("Voltage")
xlabel("Sample")

subplot(2,1,2)
plot(turb90);
title("Reflected Light Readout")
ylabel("Voltage")
xlabel("Sample")

% figure(2)
% title("Light Sensor Outputs")
% 
% subplot(2,1,1)
% plot(vis);
% title("Visible Light Readout")
% ylabel("Voltage")
% xlabel("Sample")
% 
% subplot(2,1,2)
% plot(ir);
% title("IR Light Readout")
% ylabel("Voltage")
% xlabel("Sample")
% 
% figure(3)
% plot(pressure);
% title("Pressure Sensor Output")
% ylabel("Voltage")
% xlabel("Sample")