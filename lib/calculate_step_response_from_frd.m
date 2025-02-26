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
function step_resp = calculate_step_response_from_frd(G, f_max)

    g = squeeze(G.ResponseData);
    if isnan(abs(g(1))) % Todo: interpolate based on point 2 and 3
        g(1) = g(2);
    end
    
    freq = G.Frequency;
    g(freq >= f_max & freq <= freq(end) - f_max + freq(2)) = 0;
    
    step_resp = cumsum(real(ifft(g)));

end