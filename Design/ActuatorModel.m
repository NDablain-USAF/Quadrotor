clear all
close all
clc
% Actuator Model
K = 11.93;
zeta = 8.2618;
wn = 3.6*2*pi;
b_ol = K*wn^2; % Open loop transfer function numerator
a_ol = [1 2*zeta*wn wn^2]; % Open loop transfer function denominator
z = -375; % Lead compensator zero
p = -800; % Lead compensator pole
k = 55; % Lead compensator gain
% Simulation
tf = 5;
tstep = 0.001;
t = 0:tstep:tf;

out = sim('Actuatorsim');
%%
w_measured = out.Output(:,2);
w_reference = out.Reference(:,2);
u = out.Input(:,2);
e = out.Error(:,2);

set(0, 'DefaultLineLineWidth', 1)

figure(1)
h{1} = plot(t,w_reference,'k--');
hold on
h{2} = plot(t,w_measured,'r');
legend([h{1}(1),h{2}(1)],'Desired','Measured')
ylabel('Angular Rate (rad/s)')
xlabel('Time (s)')

figure(2)
subplot(2,1,1)
plot(t,e)
ylabel('Error (rad/s)')
xlabel('Time (s)')
subplot(2,1,2)
plot(t,u);
xlabel('Time (s)')
ylabel('Control Input (PWM value)')