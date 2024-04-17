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
