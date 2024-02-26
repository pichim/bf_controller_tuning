clc, clear variables
addpath ../../bf_function_libary/
%%

Ts = 125e-6;

f_cut = 120;
[G, B, A] = get_filter('biquad', f_cut, Ts);

df = 1;
fnyq = 1/2/Ts;
freq  = linspace(df, fnyq-df, 1e3).';
g  = squeeze(freqresp(G , 2*pi*freq));
phase       = unwrap(angle(g));
group_delay = -gradient(phase, 2*pi*freq);
phase_delay = -phase ./ (2*pi*freq);

figure(1)
subplot(121)
semilogx(freq, group_delay * 1e3), grid on
xlabel('Frequency (Hz)'), ylabel('Group Delay (ms)'), title('Group Delay')
subplot(122)
semilogx(freq, phase_delay * 1e3), grid on
xlabel('Frequency (Hz)'), ylabel('Phase Delay (ms)'), title('Phase Delay')

w0 = 2*pi*f_cut;
D = 1/sqrt(2);
Gm = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, ...
    'tustin', c2dOptions('prewarp', w0));

figure(2)
subplot(121)
grpdelay(Gm.num{1}, Gm.den{1}, freq, 1/Ts), set(gca, 'XScale', 'log')
subplot(122)
phasedelay(Gm.num{1}, Gm.den{1}, freq, 1/Ts), set(gca, 'XScale', 'log')
