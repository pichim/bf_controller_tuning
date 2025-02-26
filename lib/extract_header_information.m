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
function [para, Nheader, ind] = extract_header_information(filePath) %#ok

    fid = fopen(filePath);
    tline = fgetl(fid);
    para = [];
    Nheader = 0;
    do_read_para = false;
    while ischar(tline)
    
        Nheader = Nheader + 1;
        % we start reading parameters at frameIntervalI
        if ~isempty(regexp(tline, 'frameIntervalI', 'once'))
            do_read_para = true;
        end
        % we stop reading parameters at loopIteration
        if ~isempty(regexp(tline, 'loopIteration', 'once'))
            break;
        end
        if do_read_para
            idx = regexp(tline, '",');
            para_name  = tline(2:idx-1); %#ok
            para_value = tline(idx+2:end);
            if strcmp(para_value(1), '"')
                try % 'magPID' '"40,,"'
                    eval(['para.(para_name) = [', para_value(2:end-1), '];']);
                end
            else
                eval(['para.(para_name) = [', para_value, '];']);
            end
        end
        tline = fgetl(fid);
    end
    fclose(fid);

%%

    idx = regexp(tline, ',');
    ind_cntr = 1;
    
    % handle first element, should be 'loopIteration'
    ind_name = tline(2:idx(1)-2); %#ok
    eval(['ind.(ind_name) = [', num2str(ind_cntr), '];']);
    
    % handle all elements between
    for i = 1:length(idx)-1
        ind_cntr = ind_cntr + 1;
        ind_name = tline(idx(i)+2:idx(i+1)-2);
        if strcmp(ind_name(1:4), 'eRPM')
            eval(['ind.(ind_name(1:4))(str2double((ind_name(end-1))) + 1) = [', num2str(ind_cntr), '];']);
        elseif strcmp(ind_name(end), ']')
            eval(['ind.(ind_name(1:end-3))(str2double((ind_name(end-1))) + 1) = [', num2str(ind_cntr), '];']);
        else
            eval(['ind.(ind_name) = [', num2str(ind_cntr), '];']);
        end
    end
    
    % handle last element, should be 'axisError[2]'
    ind_cntr = ind_cntr + 1;
    ind_name = tline(idx(end)+2:end-1); %#ok
    eval(['ind.(ind_name(1:end-3))(str2double((ind_name(end-1))) + 1) = [', num2str(ind_cntr), '];']);

end

