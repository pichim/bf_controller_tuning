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

match_pole = @(Ts, f_cut) exp(-Ts * 2*pi*f_cut);
inverse_warp = @(Ts, p) (1/p - 1)/(2*Ts*pi);


filter_type = 'pt1';
f_cut = 100;
Ts = 1 / 8e3;
df = 1;


% G = get_filter(filter_type, f_cut, Ts);

p = match_pole(Ts, f_cut);
f_cut_w = inverse_warp(Ts, p);
G = get_filter(filter_type, f_cut_w, Ts);


p = pole(G)
fc = -1/Ts*log(p) / (2*pi)
g0 = squeeze( freqresp(G, 2*pi*f_cut) );
m = abs(g0)
damp(G)

freq  = linspace(df, (1/2/Ts)-df, 1e3).';
g  = squeeze(freqresp(G, 2*pi*freq));
phase       = unwrap(angle(g));
group_delay = -gradient(phase, 2*pi*freq);
phase_delay = -phase ./ (2*pi*freq);


n = 2;

% G_n = get_filter(filter_type, f_cut, n * Ts);

p_n = match_pole(n*Ts, f_cut);
f_cut_w_n = inverse_warp(n*Ts, p_n);
G_n = get_filter(filter_type, f_cut_w_n, n * Ts);

% p = 1/(2*Ts*f_cut*pi + 1);
% p_n = 1/(2*n*Ts*f_cut*pi + 1);
% freq_comp = (n * log(p) ) / (log(p_n))
% % freq_comp = (n*log(1/(2*Ts*f_cut*pi + 1)))/log(1/(2*Ts*f_cut*n*pi + 1))
% G_n = get_filter(filter_type, f_cut * freq_comp, n * Ts);


p_n = pole(G_n)
fc_n = -1/(n*Ts)*log(p_n) / (2*pi)
g0_n = squeeze( freqresp(G_n, 2*pi*f_cut) );
m_n = abs(g0_n)
damp(G_n)

freq_n  = linspace(df, ( 1/2/(Ts*n))-df, 1e3).';
g_n  = squeeze(freqresp(G_n, 2*pi*freq_n));
phase_n       = unwrap(angle(g_n));
group_delay_n = -gradient(phase_n, 2*pi*freq_n);
phase_delay_n = -phase_n ./ (2*pi*freq_n);


figure(1)
bode(G, G_n, 2*pi*freq), grid on

figure(2)
subplot(121)
semilogx(freq, group_delay * 1e3), grid on, hold on
semilogx(freq_n, group_delay_n * 1e3), hold off
xlabel('Frequency (Hz)'), ylabel('Group Delay (ms)'), title('Group Delay')
subplot(122)
semilogx(freq, phase_delay * 1e3), grid on, hold on
semilogx(freq_n, group_delay_n * 1e3), hold off
xlabel('Frequency (Hz)'), ylabel('Phase Delay (ms)'), title('Phase Delay')


%% 

% syms  Ts pi f_cut n
% 
% simplify(1 - Ts*(2*pi*f_cut)/(1 + Ts*(2*pi*f_cut)))
% % 1/(2*Ts*f_cut*pi + 1)
% 
% syms p
% simplify(solve(1/(2*Ts*f_cut*pi + 1) == p, f_cut))
% % (1/p - 1)/(2*Ts*pi)
% 
% simplify( (n * log(1/(2*Ts*f_cut*pi + 1))) / (log(1/(2*n*Ts*f_cut*pi + 1))) )
% % (n*log(1/(2*Ts*f_cut*pi + 1)))/log(1/(2*Ts*f_cut*n*pi + 1))

