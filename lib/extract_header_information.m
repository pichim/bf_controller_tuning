function [para, Nheader, ind, ind_cntr] = extract_header_information(filePath) %#ok
%EXTRACT_HEADER_INFORMATION  Parse log header into parameters and column indices
%   [para, Nheader, ind, ind_cntr] = extract_header_information(filePath)
%
% PURPOSE
%   - Read a text log header and extract named parameters into a struct
%   - Build a map from column names in the first data header line to 1-based indices
%
% INPUTS
%   - filePath   char/string path to the log file to parse
%
% OUTPUTS
%   - para       struct of parameters read between 'frameIntervalI' and 'loopIteration'
%                values are parsed as numeric arrays when possible
%   - Nheader    scalar count of header lines read including the 'loopIteration' line
%   - ind        struct mapping column names to indices from the data header line
%                handles grouped names like eRPM1..eRPM4 and name[0], name[1], ...
%   - ind_cntr   scalar total number of columns discovered in the data header line
%
% METHOD
%   1) Open file and iterate line by line, incrementing Nheader for each line
%   2) Start capturing parameters after a line containing 'frameIntervalI'
%   3) Stop header scan at a line containing 'loopIteration' and keep that line as the data header
%   4) For captured parameter lines of the form:  "paramName",values
%        - extract paramName and values
%        - if values are quoted, strip quotes then evaluate as numeric list
%        - assign into para.(paramName)
%   5) Parse the data header line
%        - split by commas, starting with 'loopIteration'
%        - for names 'eRPMk' assign into ind.eRPM(k+1)
%        - for names with bracket suffix 'name[i]' assign into ind.name(i+1)
%        - otherwise assign ind.(name) = columnIndex
%
% NOTES
%   - Expects a header section followed by a CSV-style data header line beginning with 'loopIteration'
%   - Uses eval to interpret numeric lists from text; ensure the file is trusted
%   - If 'frameIntervalI' is absent, para may remain empty
%   - If 'loopIteration' is absent, parsing of column indices will fail

    fid = fopen(filePath);
    tline = fgetl(fid);
    para = [];
    Nheader = 0;
    do_read_para = false;
    while ischar(tline)

        Nheader = Nheader + 1;
        % We start reading parameters at frameIntervalI
        if ~isempty(regexp(tline, 'frameIntervalI', 'once'))
            do_read_para = true;
        end
        % We stop reading parameters at loopIteration
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

    idx = regexp(tline, ',');
    ind_cntr = 1;

    % Handle first element, should be 'loopIteration'
    ind_name = tline(2:idx(1)-2); %#ok
    eval(['ind.(ind_name) = [', num2str(ind_cntr), '];']);

    % Handle all elements between
    for i = 1:length(idx)
        ind_cntr = ind_cntr + 1;
        if i < length(idx)
            ind_name = tline(idx(i)+2:idx(i+1)-2);
        else
            ind_name = tline(idx(end)+2:end-1);
        end

        if strcmp(ind_name(1:4), 'eRPM')
            eval(['ind.(ind_name(1:4))(str2double((ind_name(end-1))) + 1) = [', num2str(ind_cntr), '];']);
        elseif strcmp(ind_name(end), ']')
            eval(['ind.(ind_name(1:end-3))(str2double((ind_name(end-1))) + 1) = [', num2str(ind_cntr), '];']);
        else
            eval(['ind.(ind_name) = [', num2str(ind_cntr), '];']);
        end
    end
end
