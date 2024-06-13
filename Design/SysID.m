clear all
close all
clc

inputHigh = 255;
inputLow = 0;
outputHigh = 1560;
outputLow = 1543;
Mag = (outputHigh-outputLow)/(inputHigh-inputLow);

inputPeak = 942;
outputPeak = 945.7;
period = 948.8-942;
phase = (360*(outputPeak-inputPeak))/period;



%
w = [0 1 2 2.25 2.37 2.5 5 10 15 17];
W = 0:0.01:50;
h = [11.93 2.14 1.17 1.05 0.97 0.94 0.44 0.16 0.05 0.06];
H = interp1(w,h,W);
ah = [0 20.9 41 62.7 51 73 112 127 165 196].*-1;
aH = interp1(w,ah,W);
Hdeb = zeros(1,length(H));
for i = 1:length(H)
    Hdeb(i) = 20*log10(H(i));
end

subplot(2,1,1)
semilogx(W.*2*pi,Hdeb)
xlabel('Frequency (rad/s)')
ylabel('Magnitude (dB)')
subplot(2,1,2)
semilogx(W.*2*pi,aH)
ylabel('Phase (deg)')

close all
k = 11.93; % DC gain
zeta = 8.2618; % Damping ratio
wn = 3.6*2*pi; % Natural Frequency
Gain = 55;
sys_ol = tf(k*wn^2,[1 2*zeta*wn wn^2]);
lead = tf([1 375],[1 800]);
sys_lead = lead*sys_ol;
num = sys_lead.Numerator{1};
num = num.*Gain;
den = sys_lead.Denominator{1};
den(4) = den(4) + num(4);
den(3) = den(3) + num(3);
sys_cl = tf(num,den);
figure(2)
margin(sys_ol)
hold on
margin(c2d(sys_cl,0.001))