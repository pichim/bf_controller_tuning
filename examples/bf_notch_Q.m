clc, clear variables
addpath ../../bf_function_libary/
%%

Ts = 125e-6;

f1 = 200; % cutoffFreq 
f0 = 230; % centerFreq 

% bf
Q = f0 * f1 / (f0 * f0 - f1 * f1) % <- Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
% Q = f0 / (2 * (f0 - f1))
% Q = (2 * (f0 - f1)) / f0

w0 = 2*pi * f0;
Gc = tf([1 0 w0^2], [1 w0/Q w0^2])

G = get_filter('notch', [f1 f0], Ts);

% damping
damp(Gc)
D = 1 / (2*Q)

freq = logspace(1, log10(1/2/Ts), 1e3);
figure(1)
bode(Gc, G, 2*pi*freq)

freq = logspace(log10(130), log10(400), 1e3);
figure(2)
bode(Gc, G, 2*pi*freq)
