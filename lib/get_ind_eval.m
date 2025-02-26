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
function ind_eval = get_ind_eval(sinarg, data, threshold)

    if nargin == 2
        threshold = 500;
    end
    
    Ndata = size(sinarg, 1);
    
    ind_eval_candidate = sinarg > 0;
    
    signal = zeros(Ndata, 1);
    signal(ind_eval_candidate) = 1;
    dsignal = [0; diff(signal)];
    
    ind_eval_start = find(dsignal >  0.9);
    ind_eval_end   = find(dsignal < -0.9) - 1;
    Neval = size(ind_eval_start, 1);
    
    ind_eval = false(Ndata, 1);
    for i = 1:Neval
        ind_verify = ind_eval_start(i):ind_eval_end(i);
        if var(data(ind_verify)) > threshold
            ind_eval(ind_verify) = true;
        end
    end

end