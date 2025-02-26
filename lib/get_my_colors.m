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
function a = get_my_colors(ind)

%% R2022a default color order

% a = [     0 0.4470 0.7410];...
%     [0.8500 0.3250 0.0980];...
%     [0.9290 0.6940 0.1250];...
%     [0.4940 0.1840 0.5560];...
%     [0.4660 0.6740 0.1880];...
%     [0.3010 0.7450 0.9330];...
%     [0.6350 0.0780 0.1840];...
%     [     0 0.4470 0.7410];...
%     [0.8500 0.3250 0.0980];...
%     [0.9290 0.6940 0.1250];...
%     [0.4940 0.1840 0.5560];...
%     [0.4660 0.6740 0.1880];...
%     [0.3010 0.7450 0.9330];...
%     [0.6350 0.0780 0.1840];...
%     [     0 0.4470 0.7410];...
%     [0.8500 0.3250 0.0980];...
%     [0.9290 0.6940 0.1250];...
%     [0.4940 0.1840 0.5560];...
%     [0.4660 0.6740 0.1880];...
%     [0.3010 0.7450 0.9330]];

%% old color order

a = [[0   0   255];...    % 1  blau
     [0   127 0  ];...    % 2  grün
     [255 0   0  ];...    % 3  rot
     [0   191 191];...    % 4  cyan
     [191 0   191];...    % 5  magenta
     [204 204 0  ];...    % 6  gelb
     [64  64  64 ];...    % 7  schwarz
     [255 153 51 ];...    % 8  orange
     [51  255 51 ];...    % 9  knallgrün
     [150 150 150];...    % 10 grau
     [255 102 178];...    % 11 rosa
     [0   170 170];...    % 12 dunkel cyan
     [170 170 0  ]]./255; % 13 dunkel gelb

%%

    if (exist('ind', 'var') && ~isempty(ind))
        a = a(ind, :);
    end  

end

