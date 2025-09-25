function dx = drone_dynamics(t, x, T)
% DRONE_DYNAMICS - model dynamiczny quadcoptera w 4 stopniach swobody
%
% x = [z; dz; phi; dphi; theta; dtheta; psi; dpsi]
% T = [T1; T2; T3; T4] - ciągi od 4 silników (N)

% ======= Parametry fizyczne =======
m  = 1.2;     % masa drona (kg)
g  = 9.81;    % przyspieszenie ziemskie (m/s^2)
l  = 0.2;     % odległość silników od środka (m)
Ix = 0.02;    % moment bezwładności względem osi X (roll)
Iy = 0.02;    % moment bezwładności względem osi Y (pitch)
Iz = 0.04;    % moment bezwładności względem osi Z (yaw)
b  = 1e-6;    % współczynnik momentu oporu śmigła (dla yaw)

% ======= Rozpakowanie stanu =======
z     = x(1);
dz    = x(2);
phi   = x(3);   % roll
dphi  = x(4);
theta = x(5);   % pitch
dtheta= x(6);
psi   = x(7);   % yaw
dpsi  = x(8);

% ======= Ciągi silników =======
T1 = T(1);
T2 = T(2);
T3 = T(3);
T4 = T(4);

% ======= Siła netto w osi Z =======
T_total = T1 + T2 + T3 + T4;

% Uwzględnienie kąta nachylenia w osi X i Y (pitch, roll)
% Zakładamy, że oś Z drona jest nachylona, więc tylko część ciągu idzie "w górę"
ddz = (T_total * cos(phi) * cos(theta) - m * g) / m;

% ======= Momenty obrotowe =======
% Roll (oś X)
tau_phi = l * (T2 - T4);
ddphi = tau_phi / Ix;

% Pitch (oś Y)
tau_theta = l * (T3 - T1);
ddtheta = tau_theta / Iy;

% Yaw (oś Z)
tau_psi = b * (T1 - T2 + T3 - T4);
ddpsi = tau_psi / Iz;

% ======= Zwrotka stanu =======
dx = zeros(8,1);
dx(1) = dz;
dx(2) = ddz;
dx(3) = dphi;
dx(4) = ddphi;
dx(5) = dtheta;
dx(6) = ddtheta;
dx(7) = dpsi;
dx(8) = ddpsi;
end
