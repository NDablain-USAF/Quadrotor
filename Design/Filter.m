clear all
close all
clc

syms phi_k0_k0 theta_k0_k0 psi_k0_k0 w_x w_y w_z dt P_k0_k0_11 P_k0_k0_12 P_k0_k0_13 P_k0_k0_21 P_k0_k0_22 P_k0_k0_23...
    P_k0_k0_31 P_k0_k0_32 P_k0_k0_33 phi_measured theta_measured psi_measured real
H = eye(3);
R = eye(3);
xhat_k0_k0 = [phi_k0_k0;theta_k0_k0;psi_k0_k0];
u = [w_x;w_y;w_z];
P_k0_k0 = [P_k0_k0_11 P_k0_k0_12 P_k0_k0_13; P_k0_k0_21 P_k0_k0_22 P_k0_k0_23; P_k0_k0_31 P_k0_k0_32 P_k0_k0_33];
Q = dt^2.*ones(3,3);
z_k1 = [phi_measured;theta_measured;psi_measured];
% Treat gyro output as input to filter
F = eye(3); % Discretized state transition matrix
B = [1 sin(phi_k0_k0)*tan(theta_k0_k0) cos(phi_k0_k0)*tan(theta_k0_k0);...
     0 cos(phi_k0_k0)            -sin(phi_k0_k0);...
     0 sin(phi_k0_k0)*sec(theta_k0_k0) cos(phi_k0_k0)*sec(theta_k0_k0)].*dt; % Discretized control input model
xhat_k1_k0 = F*xhat_k0_k0 + B*u;
P_k1_k0 = F*P_k0_k0*F' + Q;
ybar_k0 = z_k1-(H*xhat_k1_k0); % Initial residual
%%
S = H*P_k1_k0*H' + R;
K = simplify(P_k1_k0*H'*inv(S));
xhat_k1_k1 = xhat_k1_k0 + K*ybar_k0
P_k1_k1 = simplify((eye(3)-K*H)*P_k1_k0);