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
function Gds = downsample_frd(G, Ts, freq)

    % the frequency response is infinite at some frequencies due to poles on the stability boundary. 
    g = squeeze(freqresp(G, 2*pi*freq(2:end)));
    g = [g(1); g]; % copy second value to dc in case it is an integrating tf
    
    % i think this is not necessary... can't think atm
    if(isinf(g(1)))
        g(1) = g(2);
    end
    
    g(freq > 1/2/Ts) = flipud(conj(g(freq > 0 & freq < 1/2/Ts)));
    Gds = frd(g, freq, Ts, 'Units', 'Hz');

end

