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
function [Pavg, freq] = estimate_spectras(inp, window, Noverlap, Nest, Ts)

    % Todo: check if it is usefull to remove mean here
    inp = inp - mean(inp);

    [Ndata, Nsignals] = size(inp);

    % compute the frequency bins for the output x-axis
    df = 1 / (Nest * Ts);
    freq = (0:df:1/Ts-df).';
    ind = freq <= 1 / (2 * Ts);
    freq = freq(ind);
    Nfreq = length(freq);

    % factor 2 so that the magnitude corresponds to a single sided spectrum
    % 2.3*sin(2*pi*f0*time) <=> sqrt(puu(f0)) = 2.3
    W = sum(window) / Nest / 2;

    Pavg = zeros(Nfreq, Nsignals);

    for i = 1:Nsignals

        Navg = 0;

        ind_start = 1;
        ind_end   = Nest;
        Ndelta    = Nest - Noverlap;
        while ind_end <= Ndata
        
            inp_act = inp(ind_start:ind_end,i);
    
            % Todo: check if it is usefull to remove mean here
            inp_act = inp_act - mean(inp_act);
    
            inp_act = window .* inp_act;
    
            U = fft(inp_act) / (Nest * W);
            Pact = U .* conj(U);
    
            Pavg(:,i) = Pavg(:,i) + Pact(1:Nfreq);
            Navg = Navg + 1;
    
            ind_start = ind_start + Ndelta;
            ind_end   = ind_end   + Ndelta;
    
        end
    
        Pavg(:,i) = Pavg(:,i) / Navg;

    end

end
