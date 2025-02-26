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
function [Cpi, Cd, Gf, PID, para_used] = calculate_transfer_functions(para, ind_ax, throttle_avg, Ts)
    
    filter_types = {'pt1', 'biquad', 'pt2', 'pt3'};

    % Gf: y -> yf: gyro filters
    Gf = ss(tf(1, 1, Ts));
    % gyro lowpass filter 1
    if para.gyro_lowpass_hz > 0
        para_used.gyro_lowpass_hz = para.gyro_lowpass_hz;
        para_used.gyro_soft_type  = para.gyro_soft_type;
        Gf = Gf * get_filter(filter_types{para.gyro_soft_type + 1}, ...
                             para.gyro_lowpass_hz, ...
                             Ts);
    end
    % dynamic gyro lowpass filter 1
    if para.gyro_lowpass_dyn_hz(1) > 0
        % make sure Gf is 1 at start, this is not possible in current bf
        Gf = ss(tf(1, 1, Ts));
        para_used.gyro_lowpass_dyn_hz = para.gyro_lowpass_dyn_hz;
        para_used.gyro_soft_type      = para.gyro_soft_type;
        para_used.gyro_lpf_hz_avg     = get_fcut_from_exp(para.gyro_lowpass_dyn_hz(1), ...
                                                          para.gyro_lowpass_dyn_hz(2), ...
                                                          para.gyro_lowpass_dyn_expo, ...
                                                          throttle_avg);
        para_used.gyro_lpf_throttle_avg = throttle_avg;
        Gf = Gf * get_filter(filter_types{para.dterm_filter_type + 1}, ...
                             para_used.gyro_lpf_hz_avg, ...
                             Ts);
    end
    % gyro lowpass filter 2
    if para.gyro_lowpass2_hz > 0
        para_used.gyro_lowpass2_hz = para.gyro_lowpass2_hz;
        para_used.gyro_soft2_type  = para.gyro_soft2_type;
        Gf = Gf * get_filter(filter_types{para.gyro_soft2_type + 1}, ...
                             para.gyro_lowpass2_hz, ...
                             Ts);
    end
    % gyro notch filter 1
    if para.gyro_notch_hz(1) > 0
        para_used.gyro_notch_hz(1)     = para.gyro_notch_hz(1);
        para_used.gyro_notch_cutoff(1) = para.gyro_notch_cutoff(1);
        Gf = Gf * get_filter('notch', ...
                             [para.gyro_notch_cutoff(1), para.gyro_notch_hz(1)], ...
                             Ts);
    end
    % gyro notch filter 2
    if para.gyro_notch_hz(2) > 0
        para_used.gyro_notch_hz(2)     = para.gyro_notch_hz(2);
        para_used.gyro_notch_cutoff(2) = para.gyro_notch_cutoff(2);
        Gf = Gf * get_filter('notch', ...
                             [para.gyro_notch_cutoff(2), para.gyro_notch_hz(2)], ...
                             Ts);
    end
    % gyro llc
    if (isfield(para, 'gyro_llc_freq_hz'))
        if (para.gyro_llc_phase ~= 0)
            para_used.gyro_llc_freq_hz = para.gyro_llc_freq_hz;
            para_used.gyro_llc_phase   = para.gyro_llc_phase;
            Gf = Gf * get_filter('phaseComp', ...
                                 [para.gyro_llc_freq_hz, para.gyro_llc_phase], ...
                                 Ts);
        end
    end
    
    % Gd: d/dt(yf) -> d/dt(yf)f: dterm filters
    Gd = ss(tf(1, 1, Ts));
    % filter_enumeration = {'pt1', 'biquad', 'pt2', 'pt3'};
    % dterm lowpass filter 1
    if para.dterm_lpf_hz > 0
        para_used.dterm_lpf_hz      = para.dterm_lpf_hz;
        para_used.dterm_filter_type = para.dterm_filter_type;
        Gd = Gd * get_filter(filter_types{para.dterm_filter_type + 1}, ...
                             para.dterm_lpf_hz, ...
                             Ts);
    end
    % dynamic dterm lowpass filter 1
    if para.dterm_lpf_dyn_hz(1) > 0
        % make sure Gd is 1 at start, this is not possible in current bf
        Gd = ss(tf(1, 1, Ts));
        para_used.dterm_lpf_dyn_hz  = para.dterm_lpf_dyn_hz;
        para_used.dterm_filter_type = para.dterm_filter_type;
        para_used.dterm_lpf_hz_avg  = get_fcut_from_exp(para.dterm_lpf_dyn_hz(1), ...
                                                        para.dterm_lpf_dyn_hz(2), ...
                                                        para.dterm_lpf_dyn_expo, ...
                                                        throttle_avg);
        para_used.dterm_lpf_throttle_avg = throttle_avg;
        Gd = Gd * get_filter(filter_types{para.dterm_filter_type + 1}, ...
                             para_used.dterm_lpf_hz_avg, ...
                             Ts);
    end
    % dterm lowpass filter 2
    if para.dterm_lpf2_hz > 0
        para_used.dterm_lpf2_hz      = para.dterm_lpf2_hz;
        para_used.dterm_filter2_type = para.dterm_filter2_type;
        Gd = Gd * get_filter(filter_types{para.dterm_filter2_type + 1}, ...
                             para.dterm_lpf2_hz, ...
                             Ts);
    end
    % dterm notch filter
    if para.dterm_notch_hz > 0
        para_used.dterm_notch_hz     = para.dterm_notch_hz;
        para_used.dterm_notch_cutoff = para.dterm_notch_cutoff;
        Gd = Gd * get_filter('notch', ...
                             [para.dterm_notch_cutoff, para.dterm_notch_hz], ...
                             Ts);
    end
    % dterm llc
    if (isfield(para, 'dterm_llc_phase'))
        if (para.dterm_llc_phase ~= 0)
            para_used.dterm_llc_freq_hz = para.dterm_llc_freq_hz;
            para_used.dterm_llc_phase   = para.dterm_llc_phase;
            Gd = Gd * get_filter('phaseComp', ...
                                 [para.dterm_llc_freq_hz, para.dterm_llc_phase], ...
                                 Ts);
        end
    end
    

    % Gf_p: p-term filters
    Gf_p = ss(tf(1, 1, Ts));
    % pterm llc
    if (isfield(para, 'pterm_llc_phase'))
        if (para.pterm_llc_phase ~= 0)
            para_used.pterm_llc_freq_hz = para.pterm_llc_freq_hz;
            para_used.pterm_llc_phase   = para.pterm_llc_phase;
            Gf_p = Gf_p * get_filter('phaseComp', ...
                                     [para.pterm_llc_freq_hz, para.pterm_llc_phase], ...
                                     Ts);
        end
    end
    % p-term lowpass filter yaw
    if ind_ax == 3 && para.yaw_lpf_hz > 0
        para_used.yaw_lpf_hz = para.yaw_lpf_hz;
        Gf_p = Gf_p * get_filter('pt1', ...
                                  para.yaw_lpf_hz, ...
                                  Ts);
    end


    % PID parameters
    pid_axis = {'rollPID', 'pitchPID', 'yawPID'};
    if (length(para.(pid_axis{ind_ax})) == 5)
        if (para.(pid_axis{ind_ax})(3) ~= para.(pid_axis{ind_ax})(4))
            warning([pid_axis{ind_ax}, ' different D gains']);
        end
        % remove dynamic D-Term
        para.(pid_axis{ind_ax}) = para.(pid_axis{ind_ax})([1 2 3 5]);
    end
    if para.(pid_axis{ind_ax})(4) ~= 0
        warning([pid_axis{ind_ax}, ' FF is not zero']);
    end
    % Insert 0 for FF
    PID = para.(pid_axis{ind_ax}) .* [get_pid_scale(ind_ax), 0];
    

    % get controllers
    [Cpi, Cd] = calculate_controllers(PID, Gf_p, Ts);
    Cd = Cd * Gd;

end

