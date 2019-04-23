greenTeensyUnits = [];
IRTeensyUnits    = [];

greenVolts = greenTeensyUnits .* 3.3 ./ 1023;
greenCurrent = greenVolts ./ 0.3; %microAmps (300 kOhms)
greenIrradiance = greenCurrent .* 200; %Lux = 200 * microamps, from datasheet

IRVolts = IRTeensyUnits .* 3.3 ./ 1023;
IRCurrent = IRVolts ./ 2; %microAmps (2 MOhms)
IRIrradiance = IRCurrent .* 80; %Lux = 80 * microamps, from datasheet

ratioGtoIR = greenIrradiance ./ IRIrradiance;
plot(ratioGtoIR)