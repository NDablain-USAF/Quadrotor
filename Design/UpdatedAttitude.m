clear all
close all
clc
% syms Ixx Iyy Izz wx wy wz wx_dot wy_dot wz_dot Tau_x Tau_y Tau_z real
% w = [wx;wy;wz];
% I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
% u = [Tau_x;Tau_y;Tau_z];
% w_dot = I\(u - cross(w,I*w))
%%
% Time
tf = 5;
tstep = 0.0001;
t = 0:tstep:tf;
% Constants
I = eye(3);
M = [0.1;0.2;0.3];
C = [1 0 0 0 0 0;...
     0 1 0 0 0 0;...
     0 0 1 0 0 0];
% Initial conditions
x0 = [0; 0; 0]; % [phi phi_dot theta theta_dot psi psi_dot]'
P0 = zeros(3,3); % Only change this if unsure of initial states
G = [tstep;tstep;tstep];
Q = G*G';
R = eye(3);
% Simulation
out = sim('UpdatedAttitudeSim');
System = squeeze(out.Euler_true.signals.values);
Estimate = squeeze(out.Euler_estimated.signals.values);
% Plotting
figure()
subplot(3,1,1)
plot(t,System(1,:),t,Estimate(1,:))
subplot(3,1,2)
plot(t,System(2,:),t,Estimate(2,:))
subplot(3,1,3)
plot(t,System(3,:),t,Estimate(3,:))