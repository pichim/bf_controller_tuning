clc; clear variables

%% Sampling Time 
Ts      = 125 * 1.0e-6;        % Gyro loop
Ts_cntr = 2 * Ts;              % Control loop
Ts_log  = 2 * Ts_cntr;         % Logging loop (not used here, just info)
fs      = 1/Ts;

t = (0:Ts:10).';               % Time vector from 0s to 10s (column vector)
N = length(t);                 % number of samples

%% Thrust
w_thr = 5 * 2*pi;
y = 2 + sin(w_thr*t);

%% Input Signal
omega = 10.1 * 2*pi();
u = 0.6*sin(2*pi*(10 + 1*t).*t) + 0.25*randn(size(t));    %Input signal
w = hann(N, 'periodic');          % window over the entry singal
u_win = u .* w;                   % signal with window

figure(1)
subplot(311)
plot(t,u);grid on;
xlim([0 10]);
title('Input Signal');

subplot(312)
plot(t,u_win);grid on;
xlim([0 10]);
title('Input Signal with Window');

subplot(313)
plot(t, y);grid on;
xlim([0 10]);
xlabel('Time [s]');
title('Thrust');

%% Spectrogram over Time and Thrust

seg = 60;            %Number of segments
Nres = seg;
Nest = floor(N / (1+(seg-1)));  %Number of points in segment
freq(1,:) = (0:Nest/2).' * (fs / Nest);  %Get steplenght of Frequency
Nfreq = length(freq);       %Get Number of Niquist
window = hann(Nest, 'periodic');        % Window over signal

% Creation of y vector
y_min = min(y);     
y_max = max(y);
dy    = (y_max - y_min) / Nres;     %Span width of steps
y_axis = (y_min:dy:y_max-dy).';     %Creation of y vector

u_seg  = zeros(Nest, seg);      %Preparation array
t_seg  = zeros(Nest, seg);      %Preparation array
U_SEG  = zeros(Nest, seg);
U_mean  = zeros(seg);

start = 0;
stop = 0;
y_mid = zeros(seg,1);   % Thrust-Mittelwert je Segment

for i = 1:seg
    start = (i-1)*Nest + 1;  
    stop  = start + Nest - 1; 

    % Get y vektor
    y_seg = y(start:stop);      % Split up in Segments
    
    ind_y = round((y_seg - y_min) / max(y_max - y_min, eps) * (Nres - 1)) + 1;  % Split it up in different thrustlevels
    ind_y = max(1, min(Nres, ind_y));   % If thrust is under one get rid off it
    ind_y = sort(ind_y);    % Sort ind_y

    ind_y_count = zeros(size(ind_y));   % Get counter
    j = 1;
    for k = 1:length(ind_y)     % Get throw all ind y
        if ind_y(k) ~= ind_y(j) % Count how many y are the same
            j = j + 1;
            ind_y(j) = ind_y(k);
        end
        ind_y_count(j) = ind_y_count(j) + 1;    
    end

    if stop > N
        break;   % Stop when segments are full
    end
    y_mid(i) = mean(y(start:stop)); % Get mean of segment
    u_seg(:, i) = u(start:stop);    % Add Segment to array
    t_seg(:, i) = t(start:stop);
    U_SEG(:,i) = abs(fft(u_seg(:,i)));      % Absolut value of frequent component

end
% Get the step length of the t vector
t_mid = ((0:seg-1)*Nest + (Nest-1)/2) * Ts;   % 

% only positiv frequencies
U_POS    = U_SEG(1:Nfreq, :);                     % Ged rid of Alising

figure(2)
pcolor(freq, t_mid, U_POS.'); 
shading flat; set(gca,'ColorScale','log'); colormap jet; colorbar
xlabel('Frequency (Hz)'); ylabel('Time (s)'); title('Amplitude over Frequency and Time');
xlim([0 100]);

figure(3)
pcolor(freq, y_mid, U_POS.');
shading flat; set(gca,'ColorScale','log'); colormap jet; colorbar
xlabel('Frequency (Hz)'); ylabel('Thrust');
title('Amplitude over Frequency and Thrust');
xlim([0 100]); ylim([1.8 2.2])