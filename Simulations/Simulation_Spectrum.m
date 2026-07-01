clc, clear variables

%% Sampling Time 
Ts      = 125 * 1.0e-6;        % Gyro loop
Ts_cntr = 2 * Ts;              % Control loop
Ts_log  = 2 * Ts_cntr;         % Logging loop (not used here, just info)
fs      = 1/Ts;

t = (0:Ts:10).';               % Time vector from 0s to 10s (column vector)
N = length(t);                 % number of samples

%% Input Signal
omega = 10.1 * 2*pi();
u = 0.6*sin(2*pi*(10 + 0.5*t).*t) + 0.25*randn(size(t));            %Input signal
w = hann(N, 'periodic');          % window over the entry singal
u_win = u .* w;                   % signal with window

figure(1)
subplot(211)
plot(t,u);grid on;
title('Input Signal without Window');

subplot(212)
plot(t,u_win);grid on;
title('Input Sigal with Window');

%% Spectrum of Input Signal

U = fft(u);
U_WIN = fft(u_win);
w_fac = sum(w) / 2;  

% Frequenzachse (Hz)
f = (0:N-1)*(fs/N);

% Calculation of spectra
A_noWin = abs(U(1:N/2)) / (N/2);    % New vector without Niquist
A_win   = abs(U_WIN(1:N/2)) / w_fac;   %New vector with upscaling because of win
f_half  = f(1:N/2);                 % New vector without Niquist

% Plot
figure(2)
subplot(2,1,1)
plot(f_half, A_noWin)
xlabel('Frequenz [Hz]')
ylabel('Amplitude')
title('Spektrum ohne Fenster')
grid on
xlim([0 100])

subplot(2,1,2)
plot(f_half, A_win)
xlabel('Frequenz [Hz]')
ylabel('Amplitude')
title('Spektrum mit Hann-Fenster')
grid on
xlim([0 100])

%% Split up in Segments

seg = 7;            %Number of segments
overlap = 0.5;      %Overlap in next segement x*100%
Nest = floor(N / (1+(seg-1)*(1-overlap)));  %Number of points in segment
window = hann(Nest, 'periodic');        % Window over signal
Noverlap = round(overlap * Nest);       % overlap in SAMPLES for function + hop calc
Nshift   = Nest - Noverlap;             % hop size
freq = (0:Nest/2) * (fs / Nest);        %Niquistfrequence
Nfreq = length(freq);                   %Number of steps to Niquist
W = sum(window) / Nest / 2;


Nsignals = 1;
Pavg = zeros(Nfreq, Nsignals);
Pavgw = zeros(Nfreq, Nsignals);

Navg = 0;       % Counter of loop
for i = 1:seg
    
    start = (i-1)*Nshift + 1;  
    stop  = start + Nest - 1;  
    if stop > N
        break;   % Stop when segments are full
    end
    u_seg = u(start:stop);           % Fill up segment
    u_seg = u_seg - mean(u_seg);     % per-segment DC removal

    u_seg_win = u_seg .* window;     % Lay window over Segement   
    
    U_SEG     = fft(u_seg)     / (Nest/2);          % We use actually a rect windwow... so we need a upsacler
    U_SEG_WIN = fft(u_seg_win) / (sum(window)/2);   % Upscaler for window
    
    Pact = U_SEG .* conj(U_SEG); % two-sided power
    Pactw = U_SEG_WIN .* conj(U_SEG_WIN); % two-sided power

    Pseg = Pact(1:Nfreq);       
    Psegw = Pactw(1:Nfreq);

    Pseg(1)   = Pseg(1)   / 4; % DC
    Psegw(1)   = Psegw(1)   / 4; % DC

    Pseg(end) = Pseg(end) / 4; % Nyquist (exists since Nest is even)
    Psegw(end) = Psegw(end) / 4; % Nyquist (exists since Nest is even)

    Pavg  = Pavg  + Pseg;       % Count spectrum up
    Pavgw = Pavgw + Psegw;
    Navg = Navg + 1;            % Upcounter to get mean

        
end

Pavg  = Pavg  / Navg;       % Mean of Spectrum 
Pavgw = Pavgw / Navg;       % Mean of Spectrum

spectra = sqrt(Pavg);      % Go back from energie to amplitude
spectraw = sqrt(Pavgw);

figure(3)
subplot(2,1,1)
plot(freq, spectra); grid on
xlabel('Frequenz [Hz]'); ylabel('Amplitude');
title('Segmentiertes Spektrum ohne Fenster'); xlim([0 100])

subplot(2,1,2)
plot(freq, spectraw ); grid on
xlabel('Frequenz [Hz]'); ylabel('Amplitude');
title('Segmentiertes Spektrum mit Hann-Fenster'); xlim([0 100])

%% Get Spectrum with funciton

addpath ../lib/

% [pxx, freq1] = estimate_spectra(u, window, Noverlap, Nest, Ts);
% spectra_fun = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)

% figure(5)
% plot(freq1, spectra_fun); grid on;
% set(gca, 'YScale')
% title('Magnitude Spectra')
% xlabel('Frequenz [Hz]'); ylabel('Amplitude')
% xlim([0 100])
