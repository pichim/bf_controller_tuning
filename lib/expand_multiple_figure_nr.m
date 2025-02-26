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
function fig_nr = expand_multiple_figure_nr(ini_fig_nr, multp_fig_nr)
% figNr = get_multiple_figure_nr(iniFigNr, multpFigNr)
% 
% Examples:
% get_multiple_figure_nr(1,1) = 1
% get_multiple_figure_nr(2,2) = 22
% get_multiple_figure_nr(3,3) = 333

    if multp_fig_nr == 1
        fig_nr = ini_fig_nr;
        return
    else
        fig_nr_str = num2str(ini_fig_nr);
        for i = 1:multp_fig_nr-1
            fig_nr_str = [fig_nr_str, num2str(ini_fig_nr)]; %#ok
        end
    end
    fig_nr = str2double(fig_nr_str);

end

