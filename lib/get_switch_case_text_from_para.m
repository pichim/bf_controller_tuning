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
function get_switch_case_text_from_para(para)

    PID = [para.rollPID; ...
           para.pitchPID; ...
           para.yawPID];
    
    % create warnings
    pid_axis = {'roll', 'pitch', 'yaw'};
    % for i = 1:3
    %     if PID(i,3) ~= PID(i,4)
    %         warning([' different D gains in ', pid_axis{i}, ' axis ']);
    %     end
    %     if PID(i,5) ~= 0
    %         warning([' FF is not zero in ', pid_axis{i}, ' axis ']);
    %     end
    % end
    
    fprintf('        %% type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3\n');
    fprintf('        para_new.gyro_lpf            = %d;       %% dono what this is\n',       para.gyro_lpf);
    fprintf('        para_new.gyro_lowpass_hz     = %d;       %% frequency of gyro lpf 1\n', para.gyro_lowpass_hz);
    fprintf('        para_new.gyro_soft_type      = %d;       %% type of gyro lpf 1\n',      para.gyro_soft_type);
    fprintf('        para_new.gyro_lowpass_dyn_hz = [%d, %d];  %% dyn gyro lpf overwrites gyro_lowpass_hz\n', ...
                                                                                             para.gyro_lowpass_dyn_hz);
    fprintf('        para_new.gyro_lowpass2_hz    = %d;     %% frequency of gyro lpf 2\n',   para.gyro_lowpass2_hz);
    fprintf('        para_new.gyro_soft2_type     = %d;       %% type of gyro lpf 2\n',      para.gyro_soft2_type);
    fprintf('        para_new.gyro_notch_hz       = [%d, %d]; %% frequency of gyro notch 1 and 2\n', ...
                                                                                             para.gyro_notch_hz);
    gyro_notch_damp_1  = 1.0 / ( 2.0 * get_notch_Q(para.gyro_notch_hz(1), para.gyro_notch_cutoff(1)) );
    gyro_notch_damp_2  = 1.0 / ( 2.0 * get_notch_Q(para.gyro_notch_hz(2), para.gyro_notch_cutoff(2)) );
    if isnan(gyro_notch_damp_1)
        gyro_notch_damp_1 = 0;
    end
    if isnan(gyro_notch_damp_2)
        gyro_notch_damp_2 = 0;
    end
    fprintf('        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([%0.2f, %0.2f], para_new.gyro_notch_hz); %% damping of gyro notch 1 and 2\n', ...
                                                                                             [gyro_notch_damp_1, gyro_notch_damp_2]);
    
    fprintf('        para_new.dterm_lpf_hz        = %d;       %% frequency of dterm lpf 1\n', para.dterm_lpf_hz);
    fprintf('        para_new.dterm_filter_type   = %d;       %% type of dterm lpf 1\n',      para.dterm_filter_type);
    fprintf('        para_new.dterm_lpf_dyn_hz    = [%d, %d];  %% dyn dterm lpf overwrites dterm_lpf_hz\n', ...
                                                                                              para.dterm_lpf_dyn_hz);
    fprintf('        para_new.dterm_lpf2_hz       = %d;     %% frequency of dterm lpf 2\n',   para.dterm_lpf2_hz);
    fprintf('        para_new.dterm_filter2_type  = %d;       %% type of dterm lpf 2\n',      para.dterm_filter2_type);
    fprintf('        para_new.dterm_notch_hz      = %d;     %% frequency of dterm notch\n',   para.dterm_notch_hz);
    dterm_notch_damp_1 = 1.0 / ( 2.0 * get_notch_Q(para.dterm_notch_hz  , para.dterm_notch_cutoff  ) );
    if isnan(dterm_notch_damp_1)
        dterm_notch_damp_1 = 0;
    end
    fprintf('        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(%0.2f, para_new.dterm_notch_hz); %% damping of dterm notch\n', ...
                                                                                              dterm_notch_damp_1);
    fprintf('        para_new.yaw_lpf_hz          = %d;     %% frequency of yaw lpf (pt1)\n', para.yaw_lpf_hz);
    if (isfield(para, 'gyro_llc_freq_hz'))
        fprintf('        para_new.gyro_llc_freq_hz    = %d;     %% frequency of gyro llc\n', para.gyro_llc_freq_hz);
        fprintf('        para_new.gyro_llc_phase      = %d;       %% phase of gyro llc\n', para.gyro_llc_phase);
    end
    if (isfield(para, 'dterm_llc_freq_hz'))
        fprintf('        para_new.dterm_llc_freq_hz   = %d;     %% frequency of dterm llc\n', para.dterm_llc_freq_hz);
        fprintf('        para_new.dterm_llc_phase     = %d;       %% phase of dterm llc\n', para.dterm_llc_phase);
    end
    if (isfield(para, 'pterm_llc_freq_hz'))
        fprintf('        para_new.pterm_llc_freq_hz   = %d;     %% frequency of pterm llc\n', para.pterm_llc_freq_hz);
        fprintf('        para_new.pterm_llc_phase     = %d;       %% phase of pterm llc\n', para.pterm_llc_phase);
    end
    fprintf('        switch ind_ax\n');
    fprintf('            case 1 %% roll: [%d, %d, %d, %d]\n', PID(1,:));
    fprintf('                P_new       = %d;\n', PID(1,1));
    fprintf('                I_ratio_new = %d/%d;\n', PID(1,2), PID(1,2));
    fprintf('                D_new       = %d;\n', PID(1,3));
    fprintf('            case 2 %% pitch: [%d, %d, %d, %d]\n', PID(2,:));
    fprintf('                P_new       = %d;\n', PID(2,1));
    fprintf('                I_ratio_new = %d/%d;\n', PID(2,2), PID(2,2));
    fprintf('                D_new       = %d;\n', PID(2,3));
    fprintf('            case 3 %% yaw: [%d, %d, %d, %d]\n', PID(3,:));
    fprintf('                P_new       = %d;\n', PID(3,1));
    fprintf('                I_ratio_new = %d/%d;\n', PID(3,2), PID(3,2));
    fprintf('                D_new       = %d;\n', PID(3,3));
    fprintf('        end\n');

end

