clc, clear variables
addpath ../../bf_function_libary/
%%

Ts = 125e-6;
f_cut = 120;
[G, B, A] = get_filter('biquad', f_cut, Ts);

% step response
figure(1)
step(G), grid on

% arbitary input
time = (0:Ts:1.0).';
input = 0.01 * randn(size(time));
output = filter(B, A, input);

figure(2)
subplot(211)
plot(time, input), grid on
subplot(212)
plot(time, output), grid on

w0 = 2*pi*f_cut;
D = 1/sqrt(2);
Gm = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, ...
    'tustin', c2dOptions('prewarp', w0));

figure(3)
bode(G, Gm)

