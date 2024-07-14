% Arquivo principal da simulação
% ------------------------------
clear;
close all;
%% seletor de entrada
% 1     gerador de sinais
% -1    joystick
input_selector = -1;
%%
dt = 1e-3;   % passo de integração
T_sim = 100; % Tempo de simulação
% carregas as constantes do sistema
% Parametros do modelo
g = 9.8;            % aceleração da gravedade
m = 1.0428;         % massa total do veículo
Ixx = 1e-2;
Iyy = 1e-2;
Izz = 1.8e-2;
Ir = 1e-3;
Omega_r = 0;
% Transformação U1,...,U4 -> freq. angulares dos motores
kf = 1.4793*10^(-7); % constante de empuxo
km = 2.4656*10^(-9); % constante de torque
L = 0.2;
T = [kf,kf,kf,kf;
     0, L*kf,0,-L*kf;
     -L*kf,0,L*kf,0;
     -km,km,-km,km];
T_inv = T^(-1);
%% Modelo dos motores 
Kp = 0.9182;    % ganho
tau_a = 0.0569; % polo
tau_s = 0.045; % atraso de transporte
% sem limitação dos motores por enquanto
%rpm_max = 15400;
%rpm_min = 8700;
% inlcuir dinâmica dos motores?
% -1 NÃO
% 1  SIM
incluir_din_motor = 1;
%% Condições iniciais
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
%% Seleciona controlador da dinâmica rotacional
%  1 : PDI
% -1 : LQR
seleciona_controlador = -1;
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
%% A dinâmica Rotacional linear completa
% --------------------------------------
A22 = A(7:12,7:12);
B22 = B(7:12,2:4);
C = [zeros(3) eye(3)]; % a saída são os ângulos de Euler (phi, theta, psi)
D = zeros(3);
% O LQT - Linear Quadratic Tracking controller
q11 = 0.5;
q22 = 0.5;
q33 = 0.5;
Q = diag([q11,q22,q33]);
r11 = 2;
r22 = 2;
r33 = 2;
R = diag([r11,r22,r33]);
% A solução da equação algébrica de Ricatti
K = are(A22, B22*inv(R)*B22',C'*Q*C);
% A solução algébrica da edo adicional
G = inv(K*B22*inv(R)*B22'-A22')*C'*Q;
%% Controlador PID para a dinâmica rotacional
Kp_rot = 10;
Ki_rot = 1.0;
Kd_rot = 0.01;
N_rot = 100;
%% controlador PID dinâmica translacional
% sintonizado com Matlab Tunning toolbox
Kp_trans = 2.0968;
Ki_trans = 0.4;
Kd_trans = -0.13723;
N_trans = 5.5815;
