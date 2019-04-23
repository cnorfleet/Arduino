%% Initialize
% These first sections are to generate signals that simulate the diode
% signals. We also assume that the LED signal has been stored and is
% simulated byt the ref signal.
% Set sample period
clf; 
sp = 100 * 10^(-3); %seconds
sr = 1/sp; % Calculate sample rate
% Set reference frequency
f = 1.03; % Must be less than sr/2
% Generate time array
t = double(time) ./ 1000;
%t = 0:sp:100;
% Generate reference square wave using square
%ref = 0.5*(square(2*pi*f*t)+1);
ref = double(led);
plot(t,ref)
title('Plot of Reference Square Wave')
xlabel('Time (s)')
ylabel('Voltage (V)')

%% Make Signals
% Generate signals by 
% linearly changing NTU from 0 to 1000 and making noisy copies of scaled
% square wave signals, to simulate the signals coming from the two diodes.
%signal1 = (3.25-0.000414172*t*10).*awgn(ref,40,'measured'); % Note: 2nd Term is SNR in dB
%signal2 = (0.217-exp(-0.0036155*t*10+log(0.217))).*awgn(ref,40,'measured'); % Note: 2nd Term is SNR in dB
signal180 = double(turbp);
signal90 = double(turb90);% .* 3.3 ./ 1023;

% % %signal180 = signal90 .* 2; % % % remove this later

plot(t, ref, t, signal180, t, signal90)
title('Plot of Reference and Signals')
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('Reference Square Wave','180 Degree Signal','90 Degree Signal')

%% Find Freq & synthesize sine & cosine
% From here on out we are processing the simulated signals. This part is
% what the students would use to process their data.
% Figure out frequency of square wave from data.

pp = pulseperiod(ref, t);
avgFreq = 1./mean(pp)
avg180 = movmean(signal180,50);
avg90  = movmean(signal90,50);

signal180high = signal180;
signal180high(signal180high<avg180)=nan;
avgs180high = movmean(signal180high,50,'omitnan');

signal180low = signal180;
signal180low(signal180low>avg180)=nan;
avgs180low = movmean(signal180low,50,'omitnan');

signal90high = signal90;
signal90high(signal90high<avg90)=nan;
avgs90high = movmean(signal90high,50,'omitnan');

signal90low = signal90;
signal90low(signal90low>avg90)=nan;
avgs90low = movmean(signal90low,50,'omitnan');

% plot(t, signal180high)
% hold on
% plot(t, signal180low)
% plot(t, signal90high)
% plot(t, signal90low)
% hold off
% title('Plot of Window Averaged Signals')
% xlabel('Time (s)')
% ylabel('Voltage (V)')
% legend('signal180high','signal180low','signal90high','signal90low')
% figure(2)

plot(t, avgs180high)
hold on
plot(t, avgs180low)
plot(t, avgs90high)
plot(t, avgs90low)
hold off
title('Plot of Window Averaged Signals')
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('signal180high','signal180low','signal90high','signal90low')
figure(1)

%% Calculate NTU
% Take the ratio of the two signals
rat = (avgs90high-avgs90low)./(avgs180high-avgs180low);
averageRat = mean(rat)
% Calculate the NTU from the ratio
NTU = 3.821739700382864 + rat.*(939.7102998172565 + rat.*(...
    339963.5875757225 + rat.*(-8853763.781440246 + rat.*...
    87430404.18200735)));
plot(t,NTU)
title('Plot of Calculated Turbidity vs. Time')
xlabel('Time (s)')
ylabel('Turbidity (NTU)')
plot(t,rat)
title('Plot of ratio vs time')
xlabel('Time (s)')
ylabel('Ratio')
