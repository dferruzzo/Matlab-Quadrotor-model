% Arquivo principal da simulação
% ------------------------------
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
Omega_r = 0;
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
incluir_din_motor = -1;
%% Condi��es iniciais
% Condições iniciais p,q,r
p0 = 0;
q0 = 0;
r0 = 0;
% Condições iniciais ângulos de Euler
phi0 = 0;
theta0 = 0;
psi0 = 0;
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
% A din�mica Rotacional linear completa
% --------------------------------------
A22 = A(7:12,7:12);
B22 = B(7:12,2:4);
% testando a controlabilidade da dinâmica rotacional
% tem que ser 6
disp('Controlabilidade do sistema (A22,B22)');
disp(rank(ctrb(A22,B22)));
% din�mica rotacional no corpo w = (p,q,r)
Aw = A22(1:3,1:3);
Bw = B22(1:3,1:3);
Cw = eye(3);
Dw = zeros(3);
disp('Controlabilidade da din�mica (Aw,Bw)')
disp(rank(ctrb(Aw,Bw)));
%% controlador PID din�mica translacional
% sintonizado com Matlab Tunning toolbox
Kp_trans = 2.0968;%0.4;%0.5;
Ki_trans = 0.41774;%0.06;
Kd_trans = -0.13723;%0.002;
N_trans = 5.5815;%100;
%% Controlador LQR para a din�mica rotacional
q11 = 1;
q22 = 1;
q33 = 1;
q44 = 1;
q55 = 1;
q66 = 1;
Q = diag([q11,q22,q33,q44,q55,q66]);
r11 = 1;
r22 = 1;
r33 = 1;
R = diag([r11,r22,r33]);
[K,S,E] = lqr(A22,B22,Q,R);
%% Controlador PID para a din�mica rotacional
Kp_rot = 10;
Ki_rot = 1.0;
Kd_rot = 0.01;
N_rot = 100;
% Selecionar controle para a din�mica rotacional
% 1 para controle LQR
% -1 para controle PID
select_controlador = -1;
%% roda o modelo simulink
T_sim = 60;
%sim('UAV_modelo_naolinear', T_sim);
%sim('UAV_modelo_linear', T_sim);
%sim('UAV_dinamica_pqr', T_sim);