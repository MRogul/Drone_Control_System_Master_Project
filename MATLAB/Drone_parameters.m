% ======= Parametry fizyczne =======
m  = 1.2;     % masa drona (kg)
g  = 9.81;    % przyspieszenie ziemskie (m/s^2)
L  = 0.17;     % odległość silników od środka (m)
Ix = 0.02;    % moment bezwładności względem osi X (roll)
Iy = 0.02;    % moment bezwładności względem osi Y (pitch)
Iz = 0.04;    % moment bezwładności względem osi Z (yaw)
b  = 4*1e-4;    % Stała przeliczeniowa do momentu śmigła (dla yaw)
T  = 599.2;   % Siła ciągu przy throttle 60% z smigłami T5143S‑3
n  = 21098;   % Prędkość obrotowa przy throttle 60% z smigłami T5143S‑3
n_max= 29251;   %maksymalna prędkość silników

%======== Wyliczenia =============
w=2*3.14*n/60;
w_max=2*3.14*n_max/60;
kf= T*0.7/w^2 * 0.8; %80% ze względu na spadki związane z obudową śmigła

km = 1.5e-3;

  % Macierz M (3x4) - mieszacz
    M = [
        L/sqrt(2), -L/sqrt(2), -L/sqrt(2), L/sqrt(2);
        L/sqrt(2),  L/sqrt(2), -L/sqrt(2), -L/sqrt(2);
        km,        -km,         km,        -km
    ];

    % Pseudoodwrotność M (4x3)
    M_pseudo = M' * inv(M * M');
