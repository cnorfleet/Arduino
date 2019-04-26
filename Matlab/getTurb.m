function [NTU] = getTurb(turb180, turb90)

turbiditySlope     = 14471.780;
turbidityIntercept = -60.492;

signal180 = double(turb180);
signal90  = double(turb90);

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

rat = (avgs90high-avgs90low)./(avgs180high-avgs180low);
%averageRat = mean(rat);
NTU = rat .* turbiditySlope + turbidityIntercept;

end
