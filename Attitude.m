clear all
close all
clc
% Constants
Msphere = 1;
Mrotor = 1.5/4;
l = 0.2;
m = Msphere + (4*Mrotor); %kg
r = 0.1; %radius of sphere in m
Ixx = ((2/5)*Msphere*r^2) + (2*(l^2)*Mrotor); 
Iyy = Ixx;
Izz = ((2/5)*Msphere*r^2) + (4*(l^2)*Mrotor);
I = [Ixx,Iyy,Izz];
% Process
x = [0,0,0,0.01,0.01,0.01];
[A,B] = linearize2(x,Ixx,Iyy,Izz);
C = [1 0 0 0 0 0;...
    0 1 0 0 0 0;...
    0 0 1 0 0 0];
D = zeros(3,3);
% Servo Compensator Model
Abar = [zeros(3,3) C;...
    zeros(6,3) A];
Bbar = [D;B];
controllability = rank(ctrb(Abar,Bbar));
% LQR Controller
Q = 500.*diag([1 1 1 0 0 0 0 0 0]);
G = sqrt(Q)';
R = eye(3);
P = icare(Abar,Bbar,Q,R);
K = -inv(R)*Bbar'*P;
eig(Abar+Bbar*K);
% LQG
Co = [C;0 0 0 1 0 0;...
    0 0 0 0 1 0;...
    0 0 0 0 0 1];
observability = rank(obsv(A,Co));
controllability2 = rank(ctrb(A,Co'));
Qo = 0.1.*diag([1e-2 1e-2 1e-2 1e-2 1e-2 1e-2]);
Ro = diag([5e-5 5e-5 5e-5 1e-6 1e-6 1e-6]);
Plqg = icare(A,Co',Qo,Ro);
% LTR
rho = 10000;
Qf = Qo + (1/rho).*B*B';
Pltr = icare(A,Co',Qf,Ro);
L = Pltr*Co'*inv(Ro);
%% Simulation
xi = [0; pi/6; 0; 0; 0; 0];
ref = [pi/18 pi/32 0];%; 1 2 -5; 1 -2 -3; -3 0 -3;-3 0 0];
tstep = 0.001;
tf = 20;
t = 0:tstep:tf;
out = sim('Attitudesim');
U = squeeze(out.U.signals.values);
Y = squeeze(out.Y.signals.values);
X = squeeze(out.X.signals.values);
Xhat = squeeze(out.Xhat.signals.values);
Ref = out.Reference.signals.values;
% Plotting
figure(1)
subplot(3,1,1)
plot(t,X(1,:),t,Xhat(1,:),t,Ref(1).*ones(1,length(t)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Roll Angle (rad)')
legend('Process','Estimated','Reference')
subplot(3,1,2)
plot(t,X(2,:),t,Xhat(2,:),t,Ref(2).*ones(1,length(t)),'LineWidth',1.5)
xlabel('Time(s)')
ylabel('Pitch Angle (rad)')
subplot(3,1,3)
plot(t,X(3,:),t,Xhat(3,:),t,Ref(3).*ones(1,length(t)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Yaw Angle (rad)')

figure(2)
subplot(3,1,1)
plot(t,U(1,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Rolling Torque (N-m)')
subplot(3,1,2)
plot(t,U(2,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Pitching Torque (N-m)')
subplot(3,1,3)
plot(t,U(3,:),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('Yawing Torque (N-m)')