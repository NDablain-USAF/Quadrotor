clear all
close all
clc
%Setup
[l,g,m,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,g_real,m_real,Ixx_real,Iyy_real,Izz_real,Ixy_real,Ixz_real,Iyz_real,k1,k2,k1c,k2c,wmax] = constants();
con_real = [m_real,g_real,Ixx_real,Iyy_real,Izz_real,Ixy_real,Ixz_real,Iyz_real];
con = [m,g,Ixx,Iyy,Izz,Ixy,Ixz,Iyz];
I1 = ((Iyy-Izz)/Ixx);
I2 = ((Izz-Ixx)/Iyy);
I3 = ((Ixx-Iyy)/Izz);
% LQR's with servo compensators
KA = [-22.3606698191395	0.00201472423721912	-0.0210042926161882	-5.14256360667609	-0.00544994639254871	-0.00539750017780217	-0.591349567789534	2.56742275495149e-08	-0.000222035484653320
-0.00201865759740645	-22.3606792919767	0.00418645300638387	0.00545047602090562	-5.14255900978968	0.00107577976480301	2.56742275495149e-08	-0.591349306116715	-0.000195949074938586
0.0210039149580581	-0.00418834734941623	-22.3606695180018	-0.00361675577330275	-0.00107749462239836	-6.34960121069065	-0.000117956351222076	-0.000104097946061124	-0.901525846340945];
KT = [0.0719636752505308	-0.0719636752505340	3.14585832307952	0.205167084974015	-0.205167084974022	5.80096376452051	-0.215718994756261	0.215718994756262	5.40168970403702
-5.80511229906236e-05	-0.0223026286520072	-0.00508860027687580	-0.000349336428189173	-0.0738039371133473	-0.0116791225343211	0.000563771950650344	-0.123518620198918	-0.00264525417319866
0.0223026286520074	5.80511229905560e-05	-0.00508860027687579	0.0738039371133477	0.000349336428189100	-0.0116791225343211	0.123518620198919	-0.000563771950650344	-0.00264525417319865
0	0	0	0	0	0	0	0	0];
% LQG/LTR
LA = [4.47090494564492	1.24715387948497e-08	0.00499221978617721	0.744440755445128	-4.66935645274561e-05	0.00155055930580347;...
1.24715387948497e-08	4.47089657692810	5.57847618618385e-06	4.66676478608033e-05	0.744439338778690	1.57839222690215e-05;...
0.00499221978617721	5.57847618618385e-06	4.46797672976086	0.000844876774321680	-2.51353961198021e-05	1.36306650125613;...
0.0148888151089026	9.33352957216066e-07	1.68975354864336e-05	295.815260046305	5.54710117149754e-08	-0.00572824375339310;...
-9.33871290549123e-07	0.0148887867755738	-5.02707922396041e-07	5.54710117149754e-08	295.815260041503	0.00573360695386870;...
3.10111861160694e-05	3.15678445380430e-07	0.0272613300251226	-0.00572824375339310	0.00573360695386870	159.426320319228];
LT = [31.6173277650076	8.77856768873147e-05	-0.000368035074502750	1.84128362143915	3.68731318527742e-06	0.235633602584559;...
8.77856768873147e-05	31.6173277650076	0.000368035074499152	3.68731318640445e-06	1.84128362143917	-0.235633602584566;...
-0.000368035074502750	0.000368035074499152	31.6135634683629	-0.180767234820393	0.180767234820397	2.40014416703265;...
0.184128362143915	3.68731318640445e-07	-0.0180767234820393	140.096344565361	-0.000114991820838911	-1.76483720306512e-14;...
3.68731318527742e-07	0.184128362143917	0.0180767234820397	-0.000114991820838911	140.096344565361	-2.69886958431962e-15;...
0.0235633602584559	-0.0235633602584566	0.240014416703265	-1.76483720306512e-14	-2.69886958431962e-15	100.101476750307];
%% Simulation
tstep = 0.001;
tf = 10;
t = 0:tstep:tf;
x0 = [0;0;0;0;0;0;0;0;0;0;0;0];
ref = [1;1;-2];
out = sim('quadsim');
Quad = out.Quad.signals.values; % System states
Ref = out.Ref.signals.values; % Reference signal
udel = squeeze(out.udel.signals.values); % Control input delivered
udes = out.udes.signals.values'; % Control input desired
Speeds = squeeze(out.Speeds.signals.values); % Motor speeds
% Output

set(0, 'DefaultLineLineWidth', 1.5)

figure(1)
subplot(3,1,1)
h = plot(t,Quad(:,1),'r',t,ones(length(t)).*Ref(1),'k-.');
xlabel('Time (s)')
ylabel('X Displacement (m)')
legend('System', 'Command')
subplot(3,1,2)
plot(t,Quad(:,2),'r',t,ones(length(t)).*Ref(2),'k-.')
xlabel('Time (s)')
ylabel('Y Displacement (m)')
subplot(3,1,3)
plot(t,-1.*Quad(:,3),'r',t,-1.*ones(length(t)).*Ref(3),'k-.')
xlabel('Time (s)')
ylabel('Height (m)')

figure(2)
subplot(3,1,1)
plot(t,Quad(:,4),'r')
xlabel('Time (s)')
ylabel('Roll Angle (rad)')
legend('System')
subplot(3,1,2)
plot(t,Quad(:,5),'r')
xlabel('Time (s)')
ylabel('Pitch Angle (rad)')
subplot(3,1,3)
plot(t,Quad(:,6),'r')
xlabel('Time (s)')
ylabel('Yaw Angle (rad)')

figure(3)
subplot(2,2,1)
plot(t,udes(1,:),t,udel(1,:))
ylabel('Thrust (N)')
xlabel('Time (s)')
legend('Input to System','Input from Controller')
subplot(2,2,2)
plot(t,udes(2,:),t,udel(2,:))
ylabel('Rolling Torque (N-m)')
xlabel('Time (s)')
subplot(2,2,3)
plot(t,udes(3,:),t,udel(3,:))
ylabel('Pitching Torque (N-m)')
xlabel('Time (s)')
subplot(2,2,4)
plot(t,udes(4,:),t,udel(4,:))
ylabel('Yawing Torque (N-m)')
xlabel('Time (s)')

figure(4)
plot(t,Speeds(1,:),t,Speeds(2,:),t,Speeds(3,:),t,Speeds(4,:))
ylabel('Motor Speed (rpm)')
xlabel('Time (s)')
legend('Front Motor','Right Motor','Back Motor','Left Motor')