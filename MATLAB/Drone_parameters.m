clear; clc;
% ======= Parametry fizyczne =======
m  = 1.2;     % masa drona z mocowaniem ruchomym (kg)
g  = 9.81;    % przyspieszenie ziemskie (m/s^2)
L  = 0.17;     % odległość silników od środka (m)
Ix = 0.02;    % moment bezwładności względem osi X (roll)
Iy = 0.02;    % moment bezwładności względem osi Y (pitch)
Iz = 0.04;    % moment bezwładności względem osi Z (yaw)

b  = 4*1e-7;    % Stała przeliczeniowa do momentu śmigła (dla yaw)
T  = 599.2*4;   % Siła ciągu przy throttle 60% z smigłami T5143S‑3
n  = 21098;   % Prędkość obrotowa przy throttle 60% z smigłami T5143S‑3
n_max= 29251;   %maksymalna prędkość silników
a  = L/sqrt(2); %odległość silników od osi obrotów X i Y

%======== Aktuator i sensory ========
tau_m = 0.02;      % [s] ESC+silnik
tau_s = 0.015;     % [s] sensor LPF
t_delay = 0.005;   % [s] opóźnienie czujnika

%======== Komendy i odwzorowanie na rad/s ===
s_min=48; s_max=2047;         % zakres komendy (np. DShot „umowne”)
SPEED_OFFSET = 800;            % hover offset (dostrój wg hover)

omega_max = 2*pi*n_max/60;          % [rad/s]
omega_60  = 2*pi*n/60; 

s60   = s_min + 0.6*(s_max - s_min);
k_cmd2rad = (omega_max - omega_60)/(s_max - s60);
omega_min = omega_60/10 - k_cmd2rad*(s60 - s_min)/10;

T_N   = (T/1000)*g;           % [N]
% T_N   = T; 
% ======= PID =======
Ts   = 0.01;    % [s] – jak w Simulinku (Tustin)
N_d  = 10;      % współczynnik filtru D (jak w PID block)
tauD = 1/N_d;   % [s] prosta aproximacja do dyskretnego filtru D

% roll/pitch
Kp_phi=30; Ki_phi=2; Kd_phi=40;
Kp_th =30; Ki_th =2; Kd_th =40;

% yaw
Kp_psi=70; Ki_psi=5; Kd_psi=34;

% Z
Kp_z=300; Ki_z=40; Kd_z=250;

PID_LIMIT   = 250;   % ± dla att
PID_LIMIT_Z = 500;   % ± dla Z

% ======= RIG ===
balljoint_offset = 0.05;

%======== Wyliczenia =============
w=2*3.14*n/60;
w_max=2*3.14*n_max/60;
kf= T_N *0.8/w^2; %80% ze względu na spadki związane z obudową śmigła
km = 1.5e-3;
deg2rad=pi/180;
rad2deg=180/pi;


  % 
  % Macierz M (3x4) - mieszacz
  %   M = [
  %       L/sqrt(2), -L/sqrt(2), -L/sqrt(2), L/sqrt(2);
  %       L/sqrt(2),  L/sqrt(2), -L/sqrt(2), -L/sqrt(2);
  %       km,        -km,         km,        -km
  %   ];
  % 
  %   Pseudoodwrotność M (4x3)
  %   M_pseudo = M' * inv(M * M');
