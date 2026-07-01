clc, clear variables

%% Bode plot settings
% Preferences
set(cstprefs.tbxprefs, 'MagnitudeUnits', 'dB');
set(cstprefs.tbxprefs, 'FrequencyUnits', 'Hz');
set(cstprefs.tbxprefs, 'UnwrapPhase', 'Off');
set(cstprefs.tbxprefs, 'Grid', 'On');
set(groot,'defaultLineLineWidth',1.2);   % global for all new lines

% Bode options
opt = bodeoptions('cstprefs');
opt.Xlim = { [0.1 500] };      % x-axis: 0.1 Hz to 1000 Hz

%% Sampling time 
Ts      = 125 * 1.0e-6;        % Gyro loop
Ts_cntr = 2 * Ts;              % Control loop
Ts_log  = 2 * Ts_cntr;         % Logging loop (not used here, just information)
fs      = 1/Ts;

t = (0:Ts:10).';               % Time vector from 0 s to 10 s (column vector)
N = length(t);                 % Number of samples

% Frequency vector in Hz
f = (0:N-1)*(fs/N);

s = tf('s');
z = tf('z');

%% Chirp signal (logarithmic)
f0 = 0.1; f1 = 500;            % Hz
u = chirp(t, f0, 10, f1, 'logarithmic', 0);   % numeric vector
U_In = timeseries(u, t);       % optional: for Simulink/scopes

%% Interference signals

% Input interference (white noise, non-periodic)
A_l = 0.4;
omega_l = 2*pi*10;                       % kept for compatibility, not used
l = A_l * randn(size(t));                % non-periodic input noise
Int_In = timeseries(l, t);               % optional: for Simulink/scopes

% Output interference (band-limited noise, non-periodic)
A_n = 0.4;
omega_n = 2*pi*20;                       % kept for compatibility, not used
raw_noise = randn(size(t));
[b_n,a_n] = butter(4,[5 200]/(fs/2),'bandpass');  % limit to 5–200 Hz band
n = A_n * filtfilt(b_n,a_n,raw_noise);
Int_Out = timeseries(n, t);              % optional: for Simulink/scopes

%% Combined input signals
% (For MATLAB calculations use numeric vectors; timeseries above only for Simulink)
u_tilde = u + l;

%% Force everything into column form
[t, u, l, n, u_tilde] = deal(t(:), u(:), l(:), n(:), u_tilde(:));

%% Transfer function plant
P = (s+10) / ((s + 1)*(s+2));

%% Output signal
y_tilde  = lsim(P, u_tilde, t);    % system response
y        = y_tilde + n;            % add output interference

%% Plot of input
figure(1)
subplot(311)
plot(t,u);grid on;
title('Input u(t)');
subplot(312)
plot(t,l);grid on;
title('Input interference signal l(t)');
subplot(313)
plot(t,u_tilde);grid on;
title('Input u-tilde(t)');
sgtitle('Combined input signals')

%% Plot of output
figure(2)
subplot(311)
plot(t,y_tilde);grid on;
title('Output y-tilde(t)');
subplot(312)
plot(t,n);grid on;
title('Output interference signal n(t)');
subplot(313)
plot(t,y);grid on;
title('Output y(t)');
sgtitle('Combined output signals')

%% Plot of signals
figure(3)
subplot(411)
plot(t,u);grid on;
title('Input u(t)');
subplot(412)
plot(t,u_tilde);grid on;
title('Input u-tilde(t)');
subplot(413)
plot(t,y_tilde);grid on;
title('Output y-tilde(t)');
subplot(414)
plot(t,y);grid on;
title('Output y(t)');
sgtitle('Input and output signals')

%% Transfer function without adjustments (plain FFT)
U    = fft(u);
Y    = fft(y);
H1   = Y ./ U;

% Use only positive frequencies up to Nyquist
H1 = H1(1:floor(N/2));
f  = f(1:floor(N/2));

% Create FRD object for bode plot
Hfrd = frd(H1, 2*pi*f);   % bode expects rad/s
figure(4)
h = bodeplot(P, Hfrd, opt);
grid on

% ---- Set colors: plant black, Hfrd blue ----
colP   = [0 0 0];              % black
colEst = [0 0.4470 0.7410];    % MATLAB default blue

% Find all magnitude lines
magLines = findall(h, 'Type','line', 'Tag','BodeMagnitudeLine');
magLines = flipud(magLines);   % order: P, Hfrd

% Find all phase lines
phaseLines = findall(h, 'Type','line', 'Tag','BodePhaseLine');
phaseLines = flipud(phaseLines);   % order: P, Hfrd

% Plant P -> black (magnitude + phase)
set(magLines(1),   'Color', colP);
set(phaseLines(1), 'Color', colP);

% Hfrd -> blue (magnitude + phase)
set(magLines(2),   'Color', colEst);
set(phaseLines(2), 'Color', colEst);

% ---- Recreate legend with the correct lines ----
legend(magLines, {'P(s) (true)','Estimated (FFT)'}, ...
       'Location','SouthWest');

% ---- Title options as before ----
op = getoptions(h);
op.Title.String = '';
setoptions(h, op);
sgtitle('Transfer Function Y(s)/U(s)');

%% Transfer functions with signal energy (single-shot on full record)
Syy = Y .* conj(Y);
Suu = U .* conj(U);
Syu = Y .* conj(U);

th   = 1e-15 * max(Suu);     % remove extremely small numbers due to numerical errors
H2   = Syu ./ Suu;
H2(Suu < th) = NaN;          % guard
H2 = H2(1:floor(N/2));

% Create FRD object for bode plot
figure(5)
h5 = bodeplot(P, Hfrd, opt);
grid on
legend('P(s) (true)','Estimated (FFT)')
op5 = getoptions(h5);
op5.Title.String = '';
setoptions(h5, op5);
sgtitle('Transfer Function S_{yu}/S_{uu}');

%% APPLY_ROTFILTFILT (inline, from existing data)

% Baseband low-pass (zero-phase)
fc = 5;                                   % adjust if sweep speed changes
[b_lp, a_lp] = butter(4, fc/(fs/2));

% Phase (sinarg) of the logarithmic chirp from f0, f1 and total duration
Ttot = t(end) - t(1);
r    = f1 / f0;
phi  = 2*pi*f0 * ( Ttot/log(r) ) * ( r.^((t - t(1))/Ttot) - 1 );  % also known as sinarg

% Phasors
p  = exp(1i*phi);
pc = conj(p);

% Input/output signals (use actual plant input & measured output)
X_in  = u - mean(u);
X_out = y - mean(y);

% Rotate forward
in_R  = X_in  .* p;   in_Q  = X_in  .* pc;
out_R = X_out .* p;   out_Q = X_out .* pc;

% Zero-phase low-pass in baseband
in_R  = filtfilt(b_lp, a_lp, in_R);   in_Q  = filtfilt(b_lp, a_lp, in_Q);
out_R = filtfilt(b_lp, a_lp, out_R);  out_Q = filtfilt(b_lp, a_lp, out_Q);

% Back-rotate & recombine (real)
inp = real(0.5*(in_R .* pc + in_Q .* p));     % demodulated, filtered input
out = real(0.5*(out_R.* pc + out_Q.* p));     % demodulated, filtered output

%% Simple FRF from rotated signals via FFT (optional quick look)
U_rotFFT = fft(inp);
Y_rotFFT = fft(out);
H_rotFFT = Y_rotFFT ./ U_rotFFT;
H_rotFFT = H_rotFFT(1:floor(N/2));
f_axis   = (0:floor(N/2)-1)*(fs/N);
Hfrd_rotFFT = frd(H_rotFFT, 2*pi*f_axis);

figure(9)
p9 = bodeplot(P, Hfrd_rotFFT, opt);
grid on
legend('P(s) (true)','Estimated (FFT, rotated)')
op9 = getoptions(p9); op9.Title.String = ''; setoptions(p9, op9);
sgtitle('Transfer Function (FFT on rotated signals)');

%% Transfer function according to Welch (original & rotated via function)

% Use same parameters you already used above
Nest    = round(4 / Ts);        % number of samples per segment
overlap = 0.9;                  % 50% overlap

% Welch FRF for original signals u -> y
[Hfrd_welch, ~] = welch_frf(u, y, Nest, overlap, fs, true);

% Welch FRF for rotated (lock-in) signals inp -> out
[Hfrd_welch_rot, ~] = welch_frf(inp, out, Nest, overlap, fs, true);

figure(6)
p6 = bodeplot(P, Hfrd_welch, Hfrd, opt);
grid on

% ---- Define colors ----
colP     = [0 0 0];                % black
colWelch = [0 0.4470 0.7410];      % MATLAB blue
colFFT   = [1 0 0];                % red

% ---- Get magnitude and phase lines ----
magLines   = findall(p6, 'Type','line', 'Tag','BodeMagnitudeLine');
phaseLines = findall(p6, 'Type','line', 'Tag','BodePhaseLine');

% Reverse order: [P, Welch, FFT]
magLines   = flipud(magLines);
phaseLines = flipud(phaseLines);

% ---- Set colors (magnitude + phase) ----
% P(s)
set(magLines(1),   'Color', colP);
set(phaseLines(1), 'Color', colP);

% Welch
set(magLines(2),   'Color', colWelch);
set(phaseLines(2), 'Color', colWelch);

% FFT
set(magLines(3),   'Color', colFFT);
set(phaseLines(3), 'Color', colFFT);

% ---- Correctly link legend with colors ----
legend(magLines, ...
       {'P(s) (true)', 'Welch signal energy (orig)', 'Signal energy'}, ...
       'Location','SouthWest');

% ---- Options / title as before ----
op6 = getoptions(p6);
op6.Title.String = '';
setoptions(p6, op6);

sgtitle('Transfer Function (Original, Welch)');

figure(7)
p7 = bodeplot(P, Hfrd_welch_rot, Hfrd_rotFFT, opt);
grid on
legend('P(s) (true)', 'Welch signal (rotated)', 'Signal rotated', ...
       'Location','SouthWest');
op7 = getoptions(p7); op7.Title.String = ''; setoptions(p7, op7);
sgtitle('Transfer Function (Rotated/Lock-in, Welch)');

figure(10)
p10 = bodeplot(P, Hfrd, Hfrd_welch_rot, opt);
grid on

% ---- Define colors ----
colP     = [0 0 0];                % black
colOrig  = [0 0.4470 0.7410];      % blue (MATLAB default)
colRot   = [1 0 0];                % red

% ---- Get magnitude and phase lines ----
magLines   = findall(p10, 'Type','line', 'Tag','BodeMagnitudeLine');
phaseLines = findall(p10, 'Type','line', 'Tag','BodePhaseLine');

% Reverse order: [P, Welch_orig, Welch_rot]
magLines   = flipud(magLines);
phaseLines = flipud(phaseLines);

% ---- Set colors ----
% Plant (P) → black
set(magLines(1),   'Color', colP);
set(phaseLines(1), 'Color', colP);

% Welch H2 (orig) → blue
set(magLines(2),   'Color', colOrig);
set(phaseLines(2), 'Color', colOrig);

% Welch H2 (rotated) → red
set(magLines(3),   'Color', colRot);
set(phaseLines(3), 'Color', colRot);

% ---- Correctly couple legend ----
legend(magLines, ...
       {'P(s) (true)', 'Original FFT', 'Welch and rotated'}, ...
       'Location','SouthWest');

% ---- Options / title ----
op10 = getoptions(p10);
op10.Title.String = '';
setoptions(p10, op10);

sgtitle('Original vs Welch-Rotated');


%% Welch function
function [Hfrd_welch, freq] = welch_frf(u_sig, y_sig, Nest, overlap, fs, use_hann)
% WELCH_FRF  Estimate FRF via Welch method (H2 = S_yu / S_uu) and coherent mean(Y/U).
% Inputs:
%   u_sig   : input signal (column vector)
%   y_sig   : output signal (column vector)
%   Nest    : samples per segment
%   overlap : fraction [0..1) of overlap (e.g. 0.5)
%   fs      : sampling frequency [Hz]
%   use_hann: if true, use Hann window; otherwise rectangular
%
% Outputs:
%   Hfrd_welch : FRD object of H2 estimator
%   freq       : frequency vector [Hz] (0..Nyquist)

    delta = 0 * var(u_sig);      % small regularization

    u_sig = u_sig(:); y_sig = y_sig(:);     % conversion from row vector to column vector

    if use_hann                             % Decide which window will be used
        w = hann(Nest,'periodic');
    else
        w = ones(Nest,1);
    end

    Ndata = size(u_sig, 1);
    Noverlap = floor(overlap * Nest);

    % When you apply a window (like a Hann window), the signal energy is reduced
    % This is the factor to compensate for that
    denom = sum(w) / Nest / 2;          

    % Frequency axis (Hz) and single-sided length
    freq = (0:Nest/2).' * (fs / Nest);
    Nfreq = length(freq);
 
    % Global mean removal
    u_sig = u_sig - mean(u_sig);
    y_sig = y_sig - mean(y_sig);

    % Welch average of single-sided spectra [Suu, Syu, Syy]
    Pavg = zeros(Nfreq, 3);
    Navg = 0;

    ind_start = 1;
    ind_end   = Nest;
    Ndelta    = Nest - Noverlap;

    while ind_end <= Ndata
        ind = ind_start:ind_end;

        u_inp = u_sig(ind);
        y_out = y_sig(ind);

        % Per-segment mean removal
        u_seg = u_inp - mean(u_inp);
        y_seg = y_out - mean(y_out);

        % Window
        u_seg = u_seg .* w;
        y_seg = y_seg .* w;

        % FFT with single-sided normalization
        U = fft(u_seg) / (Nest * denom);
        Y = fft(y_seg) / (denom * Nest);

        % Two-sided -> single-sided (0..Nyquist), then fix DC/Nyquist
        Pact = [U.*conj(U), Y.*conj(U), Y.*conj(Y)];
        P1s  = Pact(1:Nfreq, :);
        P1s(1,  :) = P1s(1,  :) / 4;
        P1s(end,:) = P1s(end,:) / 4;

        Pavg = Pavg + P1s;
        Navg = Navg + 1;

        % Next segment
        ind_start = ind_start + Ndelta;
        ind_end   = ind_end   + Ndelta;
    end

    if Navg > 0
        Pavg = Pavg / Navg;
    end
    
    Suu = Pavg(:,1);
    Syu = Pavg(:,2);
    
    H_welch = Syu ./ (Suu + delta);  % H2 estimator with regularization
    
    Hfrd_welch = frd(H_welch, freq, 1/fs, 'Units','Hz');
end



%% Add function from Michi

% addpath ../lib/
% 
% % Linear filter for zero phase excitation filter (apply_rotfiltfilt)
% Dlp = sqrt(3) / 2;
% wlp = 2 * pi * 10;
% Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), Ts_log, 'tustin');
% 
% 
% % Lock-in clean-up (rotate, zero-phase filter in baseband, back-rotate)
% u_rf = apply_rotfiltfilt(Glp, phi, u);   % filtered input
% y_rf = apply_rotfiltfilt(Glp, phi, y);   % filtered output
% 
% % Welch parameters (use same scale as your script; ensure even Nest)
% Nest    = round(4 / Ts);        % number of samples per segment
% NoverlapS = round(0.9 * Nest);        % 90% overlap
% win       = hann(Nest,'periodic');
% delta     = 1e-12*var(u);           % tiny regularization for Suu
% 
% % FRF of variant C (function-based)
% [G_C, C_C, freq_Hz, ~] = estimate_frequency_response(u_rf, y_rf, win, NoverlapS, Nest, Ts, delta);
% 
% % --- Bode overlay: true P(s) vs. original (Welch) vs. variant C ---
% figure(11)
% pC = bodeplot(P, Hfrd_welch_rot, G_C, opt); grid on
% legend('P(s) (true)','Janick','Michi', ...
%        'Location','SouthWest');
% opC = getoptions(pC); opC.Title.String = ''; setoptions(pC, opC);
% sgtitle('Janick vs Michi');
