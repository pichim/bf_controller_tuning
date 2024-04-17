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

