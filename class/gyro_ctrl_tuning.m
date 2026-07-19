%==========================================================================
% GYRO CTRL TUNING - Betaflight Controller Analysis GYRO TUNING CLASS
%==========================================================================
% Purpose: 
%   Calculation for Gyro Tuning
%
% Authors: 
%   Yuri Bianchi
%   Janick Dort
%   Dario Jurietti
%
% Supervisors: 
%   Michael Peter
%   Prof. Dr. Ruprecht Altenburger
%
% Date: 05.06.2026
%==========================================================================
%  ADDITIONAL INFORMATION
%==========================================================================
% Estimates and calculates frequency responses for system identification
% and controller analysis.
%
% Key components analyzed:
% - T  (Complementary Sensitivity): Response from reference to output
% - P  (Plant): Response from control input to output
% - Cpi (PI Controller): Response from error to control input
% - Cd (D Controller): Response from derivative to control input
%
% Signal definitions:
% w: reference input (setpoint)
% y: system output (gyro measurements)
% u: control input (total PID output)
% v: PI controller output

classdef gyro_ctrl_tuning < handle
    properties       
        
        data_flight     % Get all data and parameters
        
        % Neede in different function
         T
        Guw
        Gvw
        P_gef
        P
        Cpi
        Cd
        Gf_ana
        Cpi_ana
        Cd_ana
        PID
        para_used
        omega_bode
        Coh_T
        Coh_C
        throttle_avg
        Cpi_ana_new
        Gf_ana_new
        Cd_ana_new
        Nest
        ind_ax
        Coh_P
        CL_ana
        CL_ana_new
        step_time
        step_resp_tra
        step_resp_com

        % Default values
        linewidth            (1,1) double  = 1.2
            
    end

     methods
         function obj = gyro_ctrl_tuning(data_flight)
            obj.data_flight = data_flight;

        end
        function obj = calculate_transfer_func(obj, Nestfatra, koverlaptra)
            dataf = obj.data_flight;

            % Analysis window parameters
            obj.Nest     = round(Nestfatra / dataf.Ts_log);    % Window length in samples
            Noverlap = floor(koverlaptra * obj.Nest);        % Overlap between windows
            window   = hann(obj.Nest, 'periodic');               % Hanning window for analysis
            
            % Design linear filter for zero phase excitation filter (apply_rotfiltfilt)
            Dlp = sqrt(3) / 2;    % Damping ratio
            wlp = 2 * pi * 10;    % Cutoff frequency [rad/s]
            Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), ...    % Discrete filter
                    dataf.Ts_log, 'tustin');                   % Using Tustin transform

            % Preallocate cells for 3 axes
            n_axes = 3;
            obj.T   = cell(1, n_axes);
            obj.Guw = cell(1, n_axes);
            obj.Gvw = cell(1, n_axes);
            obj.P   = cell(1, n_axes);
            obj.Cpi = cell(1, n_axes);
            obj.Cd  = cell(1, n_axes);
            obj.Coh_P   = cell(1, n_axes);
            obj.Coh_C = cell(1, n_axes);
            obj.P_gef = cell(1, n_axes);
            obj.Coh_T = cell(1, n_axes);

            sinarg_full = dataf.data(:, dataf.ind.sinarg);  % Copy Data to adjust it

            for ind_axis = 1:n_axes       % Calculate Transferfunction for Roll, Pitch and Yaw
            
                ind_eval = get_ind_eval( ...
                    dataf.data(:,dataf.ind.sinarg), ...
                    dataf.data(:,dataf.ind.gyroADC(ind_axis)));

                sinarg_ax = sinarg_full;
                sinarg_ax(~ind_eval) = 0;

                % Calculate average throttle
                obj.throttle_avg = median(dataf.data(ind_eval,dataf.ind.setpoint(4))) / 1.0e3;

                
                 % ----- Input signal: w (filtered setpoint for this axis) -----
                w = dataf.data(:, dataf.ind.setpoint(ind_axis));
                inp = apply_rotfiltfilt(Glp, sinarg_ax, w);
                
                % ----- Output y: filtered gyro for this axis -----
                y = dataf.data(:, dataf.ind.gyroADC(ind_axis));
                out_y = apply_rotfiltfilt(Glp, sinarg_ax, y);

                % Calculate complementary sensitivity (T) and input-output responses
                % T  , Gyw: w -> y
                [T_ax, C_T_ax] = estimate_frequency_response( ...
                    inp(ind_eval), out_y(ind_eval), window, Noverlap, obj.Nest, dataf.Ts_log);
                            
                % Calculate control sensitivity (Represents total controller output response)
                % SCw, Guw: w -> u
                u = dataf.data(:, dataf.ind.axisSum(ind_axis));
                out_u = apply_rotfiltfilt(Glp, sinarg_ax, u);
                [Guw_ax, C_Guw_ax] = estimate_frequency_response( ...
                    inp(ind_eval), out_u(ind_eval), window, Noverlap, obj.Nest, dataf.Ts_log);
                
                % Calculate PI controller response (v is the output of just the PI portion of controller)
                % Gvw: w -> v
                v = dataf.data(:, dataf.ind.axisSumPI(ind_axis));
                out_v = apply_rotfiltfilt(Glp, sinarg_ax, v);
                [Gvw_ax, ~] = estimate_frequency_response( ...
                    inp(ind_eval), out_v(ind_eval), window, Noverlap, obj.Nest, dataf.Ts_log);

                P_gef_ax = T_ax / Guw_ax;
                Coh_Plant = C_T_ax .* C_Guw_ax;   
             
                % Calculate controller frequency responses
                % Split into PI and D components for analysis
                Cpi_ax = Gvw_ax / (1 - T_ax);
                Cd_ax  = Guw_ax * Gvw_ax / T_ax * (1 / Guw_ax - 1 / Gvw_ax);

                % Prepare frequency vector for Bode plotspara
                omega_bode_ax = 2*pi*P_gef_ax.Frequency;    % Convert Hz to rad/s
                

                % ----- Store for this axis -----
                obj.T{ind_axis}   = T_ax;
                obj.Guw{ind_axis} = Guw_ax;
                obj.Gvw{ind_axis} = Gvw_ax;
                obj.P_gef{ind_axis}   = P_gef_ax;
                obj.Cpi{ind_axis} = Cpi_ax;
                obj.Cd{ind_axis}  = Cd_ax;
                obj.omega_bode = omega_bode_ax;
                obj.Coh_T{ind_axis} = C_T_ax;
                obj.Coh_C{ind_axis} = C_Guw_ax;
            
                
                % Analytical controller transfer functions for this axis
                [Cpi_ana_ax, Cd_ana_ax, Gf_ana_ax, PID_ax, para_used_ax] = ...
                    calculate_transfer_functions(dataf.para, ind_axis, obj.throttle_avg, dataf.Ts_cntr);
    
                 % Downsample to logging grid if necessary
                if Gf_ana_ax.Ts < dataf.Ts_log
                    Gf_ana_ax  = downsample_frd(Gf_ana_ax , dataf.Ts_log, P_gef_ax.Frequency);
                    Cpi_ana_ax = downsample_frd(Cpi_ana_ax, dataf.Ts_log, P_gef_ax.Frequency);
                    Cd_ana_ax  = downsample_frd(Cd_ana_ax , dataf.Ts_log, P_gef_ax.Frequency);
                end
                 % Store results per axis
                obj.Gf_ana{ind_axis}  = Gf_ana_ax;
                obj.Cpi_ana{ind_axis} = Cpi_ana_ax;
                obj.Cd_ana{ind_axis}  = Cd_ana_ax;
                obj.PID{ind_axis} = PID_ax;
                obj.para_used{ind_axis} = para_used_ax;  
                obj.P{ind_axis} = P_gef_ax / Gf_ana_ax;
                obj.Coh_P{ind_axis} = Coh_Plant;
               
            end
        end

         function obj = calculate_new_controller(obj, ind_ax, P_new, I_new, ...
                 D_new, default_parameters, para_new)

             dataf = obj.data_flight;

            tic
            
            % Initialize axis names for reporting
            pid_axis = {'rollPID', 'pitchPID', 'yawPID'};

            % Handle default parameter case
            if default_parameters
                 para_new = dataf.para;
                 pid_vec = dataf.para.(pid_axis{ind_ax});

                 % Use existing parameters
                 P_new = pid_vec(1);    % Current P gain
                 I_new = pid_vec(2);       % Keep same I gain
                 D_new = pid_vec(3);    % Current D gain
            end
            
            % Display used PID configuration
            fprintf('   used PID parameters are:\n');
            fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], ...
                dataf.para.(pid_axis{ind_ax})(1:3));
            
            % Display all used parameters (from para_used for this axis)
            pu = obj.para_used{ind_ax};          % struct with used parameters for this axis
            fields = fieldnames(pu);
            
            fprintf('   used parameters are:\n');
            for i = 1:numel(fields)
                fname = fields{i};
                val   = pu.(fname);
                fprintf('      %s: %d\n', fname, round(val));
            end

                        
            % --- Calculate New PID Parameters ---
            % Get axis-specific scaling factors (roll/pitch/yaw have different gains)
            pid_scale = [get_pid_scale(ind_ax), 1];    % Get axis scaling factors

            % Calculate new gains with scaling
            PID_new(1) = P_new * pid_scale(1);         % Scale P gain
            fI         = obj.PID{ind_ax}(2) / (2 * pi * obj.PID{ind_ax}(1));       % Extract current I frequency
            PID_new(2) = I_new *pid_scale(2);
            fI_new     = PID_new(2) / (2 * pi * PID_new(1));    % Scale I frequency
            PID_new(3) = D_new * pid_scale(3);         % Scale D gain
            PID_new(4) = 0;                                % No feedforward
            
            % Display integral frequencies
            fprintf('   used fI is: %0.2f Hz\n\n', fI);
            
            % --- Update and Display New Parameters ---
            fprintf('   new PID parameters are:\n');

            % Round scaled PID values and store in parameter structure
            para_new.(pid_axis{ind_ax}) = round( PID_new ./ pid_scale);
            para_new.(pid_axis{ind_ax}) = [para_new.(pid_axis{ind_ax})(1:3), ...
                                           para_new.(pid_axis{ind_ax})(3), ...
                                           para_new.(pid_axis{ind_ax})(4)];

            % Display new PID configuration
            fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], ...
                para_new.(pid_axis{ind_ax})(1:3));

            % --- Generate New Transfer Functions ---
            % Calculate analytical transfer functions with new parameters
            [obj.Cpi_ana_new, obj.Cd_ana_new, obj.Gf_ana_new, PID_new, para_used_new] = ...
                calculate_transfer_functions(para_new, ind_ax, obj.throttle_avg, dataf.Ts_cntr);
            
            % Display new parameter values
            para_used_fieldnames_new = fieldnames(para_used_new);
            Npara_used_new = size(para_used_fieldnames_new, 1);
            fprintf('   new parameters are:\n');
            for i = 1:Npara_used_new
                fprintf(['      ', para_used_fieldnames_new{i},': %d\n'], ...
                    eval(['round(', 'para_used_new.', para_used_fieldnames_new{i}, ');']));
            end
            
            fprintf('   new used fI is: %0.2f Hz\n\n', fI_new);
            
            % --- Process Transfer Functions ---
            % Downsample analytical controller transferfunction and convert to frd objects
            if obj.Gf_ana_new.Ts < dataf.Ts_log % by using Gf_ana.Ts we secure that we do this only once
                obj.Gf_ana_new  = downsample_frd(obj.Gf_ana_new , dataf.Ts_log, obj.P_gef{ind_ax}.Frequency);
                obj.Cpi_ana_new = downsample_frd(obj.Cpi_ana_new, dataf.Ts_log, obj.P_gef{ind_ax}.Frequency);
                obj.Cd_ana_new  = downsample_frd(obj.Cd_ana_new , dataf.Ts_log, obj.P_gef{ind_ax}.Frequency);
            end
            obj.ind_ax = ind_ax;
        end

        function obj = get_tuning_data(obj, do_compensate_iterm)

            dataf = obj.data_flight;
            % --- Calculate Closed Loop Responses ---
            % Generate closed loop responses for both current and new parameters
            obj.CL_ana     = calculate_closed_loop(obj.Cpi_ana{obj.ind_ax}    , tf(1,1,dataf.Ts_log), obj.P{obj.ind_ax}, obj.Gf_ana{obj.ind_ax}, obj.Cd_ana{obj.ind_ax});
            obj.CL_ana_new = calculate_closed_loop(obj.Cpi_ana_new, tf(1,1,dataf.Ts_log), obj.P{obj.ind_ax}, obj.Gf_ana_new, obj.Cd_ana_new);

            % Apply I-term compensation if enabled
            if do_compensate_iterm
                % Compensate only PI part
                Cpi_com = obj.Cpi{obj.ind_ax} / obj.Cpi_ana{obj.ind_ax};
    
                % Recalculate closed loop responses with compensation
                CL_ana_ = calculate_closed_loop( ...
                    obj.Cpi_ana{obj.ind_ax} * Cpi_com, ...
                    tf(1,1,dataf.Ts_log), ...
                    obj.P{obj.ind_ax}, ...
                    obj.Gf_ana{obj.ind_ax}, ...
                    obj.Cd_ana{obj.ind_ax});
    
                CL_ana_new_ = calculate_closed_loop( ...
                    obj.Cpi_ana_new * Cpi_com, ...
                    tf(1,1,dataf.Ts_log), ...
                    obj.P{obj.ind_ax}, ...
                    obj.Gf_ana_new, ...
                    obj.Cd_ana_new);
    
                % Update complementary sensitivity (nur T überschreiben)
                obj.CL_ana.T     = CL_ana_.T;
                obj.CL_ana_new.T = CL_ana_new_.T;
                
            end
            
            % Store final closed loop responses
   
            
            % =============================================================
            %  Step Response Analysis
            % =============================================================
            % Calculate and compare step responses for current and new parameters
            % Includes both tracking performance and disturbance rejection analysis
            %
            % Parameters:
            % - f_max: Maximum frequency from notch filter settings
            % - T_mean: Time window for response normalization
            % - Responses calculated for current and new parameters

            % --- Configure Analysis Parameters ---
            % Set frequency limit based on notch filter settings
            f_max = min([dataf.para.dyn_notch_min_hz, dataf.para.gyro_rpm_notch_min]);

            % Define time window for response analysis
            T_mean = 0.1 * [-1, 1] + (obj.Nest * dataf.Ts_log) / 2;    % Analysis window [s]
            obj.step_time = (0:obj.Nest-1).'*dataf.Ts_log;                 % Time vector [s]
            
            % --- Tracking Performance Analysis ---
            % Calculate step responses for setpoint tracking
            step_resp = [calculate_step_response_from_frd(obj.CL_ana.T    , f_max), ...    % Used parameters
                         calculate_step_response_from_frd(obj.CL_ana_new.T, f_max), ...    % New parameters
                         calculate_step_response_from_frd(obj.T{obj.ind_ax}, f_max)];       % Measured response

            % Normalize responses around their mean values
            step_resp_mean = mean(step_resp(obj.step_time > T_mean(1) & obj.step_time < T_mean(2),:));
            step_resp = step_resp ./ step_resp_mean;
            % Store tracking response results
            obj.step_resp_tra = step_resp;
           
            % --- Disturbance Rejection Analysis ---
            % Calculate responses to disturbance inputs
            step_resp = [calculate_step_response_from_frd(obj.CL_ana.SP    , f_max), ...    % Used parameters
                         calculate_step_response_from_frd(obj.CL_ana_new.SP, f_max)];       % New parameters
            
            % Center responses around their mean
            step_resp_mean = mean(step_resp(obj.step_time > T_mean(1) & obj.step_time < T_mean(2),:));
            step_resp = step_resp - step_resp_mean;
            % Store disturbance response results
            obj.step_resp_com = step_resp;                     
            toc
            
        end

   end
end
