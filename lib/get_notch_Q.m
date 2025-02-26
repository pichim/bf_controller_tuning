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
function Q = get_notch_Q(centerFreq, cutoffFreq)
% get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
% Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
    Q = centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);

end