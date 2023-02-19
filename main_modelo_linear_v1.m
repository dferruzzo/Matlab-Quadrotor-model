% Modelo linear
clear
close all
%% Parametros da simulação
g = 9.8;    % aceleração da gravedade
m = 1;      % massa total do veículo
Ixx = 1e-2;
Iyy = 1e-2;
Izz = 1.8e-2;
Ir = 1e-2;
Omega_r = 0;
%% sistema linear completo
A = zeros(12);
A(1,2) = 1;
A(2,11) = g;
A(3,4) = 1;
A(4,10) = -g;
A(5,6) = 1;
A(7,8) = -Ir*Omega_r/Ixx;
A(8,7) = Ir*Omega_r/Iyy;
A(10,7) = 1;
A(11,8) = 1;
A(12,9) = 1;
%
B = zeros(12,4);
B(6,1) = 1/m;
B(7,2) = 1/Ixx;
B(8,3) = 1/Iyy;
B(9,4) = 1/Izz;
% A dinâ¢mica Rotacional linear completa
% --------------------------------------
A22 = A(7:12,7:12);
B22 = B(7:12,2:4);
% dinêmica rotacional no corpo w = (p,q,r)
Aw = A22(1:3,1:3);
Bw = B22(1:3,1:3);
Cw = eye(3);
Dw = zeros(3);
%% Controlador translacional PID;
vxd = 0;
vyd = 0;
vzd = 0.001-m*g;
Kp_trans = 1;
Ki_trans = 0;
Kd_trans = 0;
%% Controlador Rotacional PID;
Kp_rot = 1;
Ki_rot = 0.5;
Kd_rot = 0;
%% Simulação simulink
x0 = 0;
y0 = 0;
z0 = 1;
p0 = 0;
q0 = 0;
r0 = 0;
phi0 = 0;
theta0 = 0;
psi0 = 0;
% tempo de simulação
T_sim = 100;