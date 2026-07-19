function [Cpi, Cd, Gf, PID, para_used] = calculate_transfer_functions(para, ind_ax, throttle_avg, Ts)
%CALCULATE_TRANSFER_FUNCTIONS  Build controller filters and gains from parameter struct
%   [Cpi, Cd, Gf, PID, para_used] = calculate_transfer_functions(para, ind_ax, throttle_avg, Ts)
%
% PURPOSE
%   - Assemble gyro path filters Gf, D-term filters Gd, and P-term path filter Gf_p
%   - Construct discrete-time PI and D controllers for the selected axis
%   - Return effective PID gains vector and a record of parameters actually used
%
% INPUTS
%   - para           struct with filter and PID settings (gyro, dterm, pterm, notches, LLC, etc)
%   - ind_ax         axis index 1=roll, 2=pitch, 3=yaw
%   - throttle_avg   scalar in [0,1] for dynamic filter cutoff computation
%   - Ts             sample time in seconds
%
% OUTPUTS
%   - Cpi        state-space PI(+P) controller including P-path filter Gf_p
%   - Cd         state-space D controller including D-term filter stack Gd
%   - Gf         state-space overall gyro-path filter
%   - PID        [Kp Ki Kd FF] effective gains vector with axis scaling applied, FF forced to 0
%   - para_used  struct subset of para fields actually instantiated into filters
%
% SCALING & CONVENTIONS
%   - Axis names: {'rollPID','pitchPID','yawPID'}, selected by ind_ax
%   - If a 5-element PID is provided [Kp Ki Kd Kd_dyn FF], dynamic D is removed and FF checked
%   - Gains are scaled by get_pid_scale(ind_ax) before forming controllers
%
% METHOD
%   1) Start with identity Gf, Gd, Gf_p
%   2) Append enabled gyro filters to Gf: lowpass1, dynamic lowpass1, lowpass2, notches, LLC
%   3) Append enabled D-term filters to Gd: lowpass1, dynamic lowpass1, lowpass2, notch, LLC
%   4) Append enabled P-path filter(s) to Gf_p: LLC, optional yaw PT1 lowpass
%   5) Extract axis PID from para, remove dynamic D, zero FF, apply axis scaling to [Kp Ki Kd]
%   6) Form controllers via calculate_controllers: Cpi = Kp*Gf_p + Ki*Ts*z/(z-1), Cd = (Kd/Ts)*(1 - z^-1)/z^-1
%   7) Cascade Cd with Gd to include D-term filters in the derivative path
%
% NOTES
%   - get_filter selects among {'pt1','biquad','pt2','pt3','notch','phaseComp'}
%   - Dynamic LPF cutoffs are computed by get_fcut_from_exp using throttle_avg
%   - All filters and controllers are returned as discrete-time state-space models with sample time Ts

    filter_types = {'pt1', 'biquad', 'pt2', 'pt3'};

    % Gf: y -> yf: gyro filters
    Gf = ss(tf(1, 1, Ts));
    % Gyro lowpass filter 1
    if para.gyro_lowpass_hz > 0
        para_used.gyro_lowpass_hz = para.gyro_lowpass_hz;
        para_used.gyro_soft_type  = para.gyro_soft_type;
        Gf = Gf * get_filter(filter_types{para.gyro_soft_type + 1}, ...
                             para.gyro_lowpass_hz, ...
                             Ts);
    end
    % Dynamic gyro lowpass filter 1
    if para.gyro_lowpass_dyn_hz(1) > 0
        % Make sure Gf is 1 at start, this is not possible in current bf
        Gf = ss(tf(1, 1, Ts));
        para_used.gyro_lowpass_dyn_hz = para.gyro_lowpass_dyn_hz;
        para_used.gyro_soft_type      = para.gyro_soft_type;
        para_used.gyro_lpf_hz_avg     = get_fcut_from_exp(para.gyro_lowpass_dyn_hz(1), ...
                                                          para.gyro_lowpass_dyn_hz(2), ...
                                                          para.gyro_lowpass_dyn_expo, ...
                                                          throttle_avg);
        para_used.gyro_lpf_throttle_avg = throttle_avg;
        Gf = Gf * get_filter(filter_types{para.gyro_soft_type + 1}, ...
                             para_used.gyro_lpf_hz_avg, ...
                             Ts);



    end
    % Gyro lowpass filter 2
    if para.gyro_lowpass2_hz > 0
        para_used.gyro_lowpass2_hz = para.gyro_lowpass2_hz;
        para_used.gyro_soft2_type  = para.gyro_soft2_type;
        Gf = Gf * get_filter(filter_types{para.gyro_soft2_type + 1}, ...
                             para.gyro_lowpass2_hz, ...
                             Ts);
    end
    % Gyro notch filter 1
    if para.gyro_notch_hz(1) > 0
        para_used.gyro_notch_hz(1)     = para.gyro_notch_hz(1);
        para_used.gyro_notch_cutoff(1) = para.gyro_notch_cutoff(1);
        Gf = Gf * get_filter('notch', ...
                             [para.gyro_notch_cutoff(1), para.gyro_notch_hz(1)], ...
                             Ts);
    end
    % Gyro notch filter 2
    if para.gyro_notch_hz(2) > 0
        para_used.gyro_notch_hz(2)     = para.gyro_notch_hz(2);
        para_used.gyro_notch_cutoff(2) = para.gyro_notch_cutoff(2);
        Gf = Gf * get_filter('notch', ...
                             [para.gyro_notch_cutoff(2), para.gyro_notch_hz(2)], ...
                             Ts);
    end
    % Gyro llc
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
    % Dterm lowpass filter 1
    if para.dterm_lpf_hz > 0
        para_used.dterm_lpf_hz      = para.dterm_lpf_hz;
        para_used.dterm_filter_type = para.dterm_filter_type;
        Gd = Gd * get_filter(filter_types{para.dterm_filter_type + 1}, ...
                             para.dterm_lpf_hz, ...
                             Ts);
    end
    % Dynamic dterm lowpass filter 1
    if para.dterm_lpf_dyn_hz(1) > 0
        % Make sure Gd is 1 at start, this is not possible in current bf
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
    % Dterm lowpass filter 2
    if para.dterm_lpf2_hz > 0
        para_used.dterm_lpf2_hz      = para.dterm_lpf2_hz;
        para_used.dterm_filter2_type = para.dterm_filter2_type;
        Gd = Gd * get_filter(filter_types{para.dterm_filter2_type + 1}, ...
                             para.dterm_lpf2_hz, ...
                             Ts);
    end
    % Dterm notch filter
    if para.dterm_notch_hz > 0
        para_used.dterm_notch_hz     = para.dterm_notch_hz;
        para_used.dterm_notch_cutoff = para.dterm_notch_cutoff;
        Gd = Gd * get_filter('notch', ...
                             [para.dterm_notch_cutoff, para.dterm_notch_hz], ...
                             Ts);
    end
    % Dterm llc
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
    % Pterm llc
    if (isfield(para, 'pterm_llc_phase'))
        if (para.pterm_llc_phase ~= 0)
            para_used.pterm_llc_freq_hz = para.pterm_llc_freq_hz;
            para_used.pterm_llc_phase   = para.pterm_llc_phase;
            Gf_p = Gf_p * get_filter('phaseComp', ...
                                     [para.pterm_llc_freq_hz, para.pterm_llc_phase], ...
                                     Ts);
        end
    end
    % P-term lowpass filter yaw
    if ind_ax == 3 && para.yaw_lpf_hz > 0
        para_used.yaw_lpf_hz = para.yaw_lpf_hz;
        Gf_p = Gf_p * get_filter('pt1', ...
                                  para.yaw_lpf_hz, ...
                                  Ts);
    end

    % PID parameters
    pid_axis = {'rollPID', 'pitchPID', 'yawPID'};
    if (length(para.(pid_axis{ind_ax})) == 5)
        if (para.(pid_axis{ind_ax})(3) ~= para.(pid_axis{ind_ax})(4) && ...
                para.(pid_axis{ind_ax})(4) ~= 0)
            warning([pid_axis{ind_ax}, ' different D gains']);
        end
        % Remove dynamic D-Term
        para.(pid_axis{ind_ax}) = para.(pid_axis{ind_ax})([1 2 3 5]);
    end
    if para.(pid_axis{ind_ax})(4) ~= 0
        warning([pid_axis{ind_ax}, ' FF is not zero']);
    end
    % Insert 0 for FF
    PID = para.(pid_axis{ind_ax}) .* [get_pid_scale(ind_ax), 0];


    % Get controllers
    [Cpi, Cd] = calculate_controllers(PID, Gf_p, Ts);
    Cd = Cd * Gd;

end
