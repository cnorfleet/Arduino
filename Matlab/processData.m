filenum = '006';
logreaderFunct

pressure = A15;
led      = A16;
turb180  = A00;
turb90   = A01;
vis      = A02;
ir       = A03;

t     = double(time) ./ 1000;     % in seconds
depth = A15 .* 1.524 - 210.13;    % in cm
NTU   = getTurb(turb180, turb90); % in NTU

figure(1)
plot3(x,y,-depth,"rx")
hold on
plot3(x,y,-depth,"b-")
hold off
title('Path of Robot During Deployment')
xlabel('x Location (m)')
ylabel('y Location (m)')
zlabel('depth (cm)')

figure(2)
plot(t, -depth);
title("Pressure Sensor Output")
ylabel("Depth (cm)")
xlabel("Time (s)")

figure(3)
plot(t,NTU)
title('Measured Turbidity Levels')
xlabel('Time (s)')
ylabel('Turbidity (NTU)')

figure(4)
movavgIR  = movmean(ir, 10);
movavgVis = movmean(vis,10);
plot(t, movavgIR)
hold on
plot(t, movavgVis)
hold off
title('Measured Light Levels')
xlabel('Time (s)')
ylabel('Light Level (teensy units)')
legend('IR', 'Vis')
%lightsensor_callibration