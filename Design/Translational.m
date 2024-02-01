clear all
close all
clc
% Constants
m = 2.5;
g = 9.81;
% Process
x = [0;0;0;0;0;0];
u = [0;pi/32;pi/32;0];
[A,B] = linearize(x,u,m,g);
C = eye(6);
D = zeros(4,4);
% Servo Compensator Model
Cc = [1 0 0 0 0 0;...
    0 1 0 0 0 0;...
    0 0 1 0 0 0];
Dc = zeros(3,4);
Abar = [zeros(3,3) Cc;...
    zeros(6,3) A];
Bbar = [Dc;B];
controllability = rank(ctrb(Abar,Bbar));
% LQR controller
Q = diag([1 1 10 0 0 0 0 0 0]);
G = sqrt(Q)';
R = diag([1 2000 2000 2000]);
P = icare(Abar,Bbar,Q,R);
k = -inv(R)*Bbar'*P;
% LQG
observability = rank(obsv(A,C));
controllability2 = rank(ctrb(A,C'));
Qo = 10.*eye(6);
Ro = diag([1e-2 1e-2 1e-2 1e-3 1e-3 1e-3]);
Plqg = icare(A,C',Qo,Ro);
% LTR
rho = 10;
Qf = Qo + (1/rho).*B*B';
Pltr = icare(A,C',Qf,Ro);
L = Pltr*C'*inv(Ro);
%% Simulation
xi = [0; 0; 0; 0; 0; 0];
ref = [0 0 -2];
tstep = 0.001;
tf = 25;
t = 0:tstep:tf;
out = sim('Translationalsim');
U = squeeze(out.U.signals.values);
Y = squeeze(out.Y.signals.values);
X = squeeze(out.X.signals.values);
Xhat = squeeze(out.Xhat.signals.values);
Yhat = squeeze(out.Yhat.signals.values);
%% Plotting
figure(1)
subplot(3,1,1)
plot(t,X(1,:),t,Xhat(1,:),t,ref(1).*ones(1,length(t)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Xn Displacement (m)')
legend('Process','Estimated','Reference')
subplot(3,1,2)
plot(t,X(2,:),t,Xhat(2,:),t,ref(2).*ones(1,length(t)),'LineWidth',1.5)
xlabel('Time(s)')
ylabel('Xe Displacement (m)')
subplot(3,1,3)
plot(t,-X(3,:),t,-Xhat(3,:),t,-ref(3).*ones(1,length(t)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Height (m)')

figure(2)
subplot(2,2,1)
plot(t,U(1,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Thrust (N)')
subplot(2,2,2)
plot(t,U(2,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Roll Angle (rad)')
subplot(2,2,3)
plot(t,U(3,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Pitch Angle (rad)')
subplot(2,2,4)
plot(t,U(4,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Yaw Angle (rad)')
