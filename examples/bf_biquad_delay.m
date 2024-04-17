%
% This file is part of pichim's controller tuning framework.
%
% This sofware is free. You can redistribute this software
% and/or modify this software under the terms of the GNU General
% Public License as published by the Free Software Foundation,
% either version 3 of the License, or (at your option) any later
% version.
%
% This software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%
% See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public
% License along with this software.
%
% If not, see <http:%www.gnu.org/licenses/>.
%
%%
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
