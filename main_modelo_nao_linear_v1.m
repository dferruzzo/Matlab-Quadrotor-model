% Arquivo principal da simula��o
% ------------------------------
% Modelo 1
% --------
clear;
close all;
%%
% carregas as constantes do sistema
% Parametros da simulação
g = 9.8;    % aceleração da gravedade
m = 1.0428;      % massa total do veículo
Ixx = 1e-2;
Iyy = 1e-2;
Izz = 1.8e-2;
Ir = 1e-3;
% Transforma��o U1,...,U4 -> freq. angulares dos motores
kf = 1.4793*10^(-7); % constante de empuxo
km = 2.4656*10^(-9); % constante de torque
L = 0.2;
T = [kf,kf,kf,kf;
     0, L*kf,0,-L*kf;
     -L*kf,0,L*kf,0;
     -km,km,-km,km];
%T = eye(4); 
T_inv = T^(-1);
%% Modelo dos motores 
Kp = 0.9182;    % ganho
tau_a = 0.0569; % polo
tau_s = 0.0452; % atraso de transporte
% sem limita��o dos motores por enquanto 08/11/2022
%rpm_max = 15400;
%rpm_min = 8700;
% inlcuir din�mica dos motores?
% -1 N�O
% 1  SIM
incluir_din_motor = 1;
%% Condi��es iniciais
% Condições iniciais ângulos de Euler
phi0 = 0;
theta0 = 0;
psi0 = 0;
% Condições iniciais derivada dos �ngulos de Euler
dphi0 = 0;
dtheta0 = 0;
dpsi0 = 0;
% Condições iniciais para as velocidades translacionais
vx0 = 0;
vy0 = 0;
vz0 = 0;
% Condições iniciais para as posições
x0 = 0;
y0 = 0;
z0 = 1;
%% Parâmetros
% velocidade desejada
vxd = 0;
vyd = 0;
vzd = 0;
%% controlador PID din�mica translacional
Kp_trans = 0.4;%0.5;
Ki_trans = 0.06;
Kd_trans = 0.002;
%% Controlador PID para a din�mica rotacional
Kp_rot = 10;
Ki_rot = 1.0;
Kd_rot = 0.01;
%% roda o modelo simulink
T_sim = 60;
%sim('UAV_modelo_naolinear_1', T_sim);