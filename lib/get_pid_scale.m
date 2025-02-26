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
function pid_scale = get_pid_scale(ind_ax)

    PTERM_SCALE = 0.032029;
    ITERM_SCALE = 0.244381;
    DTERM_SCALE = 0.000529;
    % FEEDFORWARD_SCALE = 0.013754;
    
    pid_scale = [PTERM_SCALE, ITERM_SCALE, DTERM_SCALE];
    
    if ind_ax == 3
        pid_scale(2) = pid_scale(2) * 2.5;
    end

end

