fname = 'dane.csv';
%vars  = {'time','ac_x','ac_y','gyr_x','gyr_y'};
vars  = {'time','ac_x','ac_y','ac_x_f','ac_y_f','gyr_x','gyr_y'};

opts = detectImportOptions(fname, ...
    'Delimiter',';', ...          % jeśli masz ; między kolumnami
    'DecimalSeparator', ',', ...  % liczby typu 3,14
    'ThousandsSeparator',' ');    % opcjonalnie: spacja jako separator tysięcy

% (opcjonalnie) ogranicz się do konkretnych kolumn:
opts.SelectedVariableNames = vars;

T = readtable(fname, opts);   % liczby powinny przyjść jako double


t = T.time;                         % czas [s] (jeśli nie masz, patrz niżej)
x = T.ac_x                        % wybierz interesujący sygnał

% Jeśli nie masz kolumny czasu: ustaw fs ręcznie i zrób wektor czasu
% fs = 1000;                 % Hz, RZECZYWISTE próbkowanie!
% t = (0:height(T)-1).' / fs;

% Z samych danych wylicz fs (gdy t jest nierówno próbkowane, weź średnie dt)
fs = 1/mean(diff(t));
x = detrend(x,'constant');       % usuń offset


N  = length(x);
w  = hann(N);
X  = fft(x .* w);
f  = (0:N-1)*(fs/N);             % wektor częstotliwości 0..fs-1/N

% Widmo jednostronne (amplituda)
X1 = X(1:floor(N/2)+1);
f1 = f(1:floor(N/2)+1);
A1 = abs(X1) * 2 / sum(w);       % skala ~ amplitudowa (okno uwzględnione)

% Faza (w stopniach)
phi = angle(X1) * 180/pi;

figure; 
subplot(2,1,1); 
semilogx(f1, A1); grid on;
xlabel('Częstotliwość [Hz]'); ylabel('|X(f)|');
title('FFT — amplituda (jednostronne)');

subplot(2,1,2);
semilogx(f1, phi); grid on;
xlabel('Częstotliwość [Hz]'); ylabel('Faza [deg]');


nfft = 1024;              % długość FFT → Δf ≈ 0.1 Hz
win = hanning(512);       % okno o długości 512 próbek = 5.12 s
noverlap = 256;           % 50% nakładania

[pxx,f] = pwelch(x, win, noverlap, nfft, fs);    % pxx: jednostki sygnału^2/Hz

figure;
plot(f, 10*log10(pxx)); grid on;
xlabel('Częstotliwość [Hz]'); ylabel('PSD [dB/Hz]');
title('Gęstość mocy (Welch)');
xlim([0 fs/2]);

