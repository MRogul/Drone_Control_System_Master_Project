m = 1.0;       % masa w kg
g = 9.81;      % grawitacja
L = 0.165;     % długość ramienia
Jx = 0.02;     % momenty bezwładności
Jy = 0.02;
Jz = 0.04;
b  = 1e-6;     % współczynnik yaw (przybliżony)


z_ddot = (u1 * cos(φ) * cos(θ) - m * g) / m;

phi_ddot   = u2 / Jx;
theta_ddot = u3 / Jy;
psi_ddot   = u4 / Jz;

u1 = T1 + T2 + T3 + T4;
u2 = L * (T4 - T2);      % roll
u3 = L * (T3 - T1);      % pitch
u4 = b * (T1 - T2 + T3 - T4);   % yaw (b to stała momentu oporu)
