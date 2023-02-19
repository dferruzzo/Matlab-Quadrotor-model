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
Aw = zeros(6,6);
Aw(4,1)=1;
Aw(5,2)=1;
Aw(6,3)=1;
Bw = zeros(6,3);
Bw(1,1)=1/Ixx;
Bw(2,2)=1/Iyy;
Bw(3,3)=1/Izz;
Cw = eye(6);
Dw = zeros(6,3);
%% Controlador translacional PID;
vxd = 0;
vyd = 0;
vzd = 0;
Kp_trans = 2.0;
Ki_trans = 1.0;
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