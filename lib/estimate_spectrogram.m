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
function [Pavg, freq, y_axis] = estimate_spectrogram(inp, y, window, Noverlap, Nest, Nres, Ts)

    % Todo: check if it is usefull to remove mean here
    % inp = inp - mean(inp);
    % y   = y   - mean(y);

    Ndata = size(inp, 1);

    % find the minimum and maximum values of y and generate linearly spaced vector for the output y-axis
    y_min = min(y);
    y_max = max(y);
    dy = (y_max - y_min) / Nres;
    y_axis = (y_min:dy:y_max-dy).';

    % compute the frequency bins for the output x-axis
    df = 1 / (Nest * Ts);
    freq = (0:df:1/Ts-df).';
    ind = freq <= 1 / (2 * Ts);
    freq = freq(ind);
    Nfreq = length(freq);

    % factor 2 so that the magnitude corresponds to a single sided spectrum
    % 2.3*sin(2*pi*f0*time) <=> sqrt(puu(f0)) = 2.3
    W = sum(window) / Nest / 2;

    Pavg = zeros(Nres, Nfreq);
    Navg = zeros(Nres, 1);

    ind_start = 1;
    ind_end   = Nest;
    Ndelta    = Nest - Noverlap;

    while ind_end <= Ndata

        inp_act = inp(ind_start:ind_end);
    
        % Todo: check if it is usefull to remove mean here
        % inp_act = inp_act - mean(inp_act);

        inp_act = window .* inp_act;

        U = fft(inp_act) / (Nest * W);
        Pact = U .* conj(U);

        % map y values to indices in the spectrogram resolution
        ind_y = sort( round((y(ind_start:ind_end) - y_min) / (y_max - y_min) * (Nres - 1)) + 1 );
        ind_y_count = zeros(size(ind_y));
        j = 1;
        for i = 1:length(ind_y)
            if (ind_y(i) ~= ind_y(j))
                j = j + 1;
                ind_y(j) = ind_y(i);
            end
            ind_y_count(j) = ind_y_count(j) + 1;
        end

        % accumulate the magnitudes into the spectrogram matrix
        for i = 1:j
            Pavg(ind_y(i), :) = Pavg(ind_y(i), :) + ind_y_count(i) * Pact(1:Nfreq).';
            Navg(ind_y(i)) = Navg(ind_y(i)) + ind_y_count(i);
        end

        ind_start = ind_start + Ndelta;
        ind_end   = ind_end   + Ndelta;

    end

    Pavg(Navg ~= 0,:) = Pavg(Navg ~= 0,:) ./ Navg(Navg ~= 0);

end
