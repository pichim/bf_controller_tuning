%==========================================================================
% ANGLE CTRL TUNING - Betaflight Controller Analysis ANGLE TUNING CLASS
%==========================================================================
% Purpose: 
%   Calculation for Angle Tuning
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


classdef angle_ctrl_tuning < handle
    properties
        data_flight     % Get all data and parameters
        gyro_tuning     % Get values of the inner cascade
        TargetAngle
    
        % Needed in different functions
        T
        P
        C
        T_gy
        Coh_T
        Coh_P
        Coh_C
        
        % Filters / controller
        C_ana_new
        C_ana

        % Axis
        ind_ax
    
    
        % analysis helpers
        omega_bode
        Nest
        step_time
    
        % closed-loop results
        CL_ana
        CL_ana_new
    
        % step response
        step_resp_tra
        step_resp_com

    end
    methods

        function obj = angle_ctrl_tuning(data_flight, gyro_tuning)

            obj.data_flight = data_flight;
            obj.gyro_tuning = gyro_tuning;
        end

       function obj = calculate_Angle_trans(obj, Nestfatra, koverlaptra)
            dataf = obj.data_flight;
        
            % Analysis window parameters
            obj.Nest   = round(Nestfatra / dataf.Ts_log);
            Noverlap   = floor(koverlaptra * obj.Nest);
            window     = hann(obj.Nest, 'periodic');
        
            % Excitation filter
            Dlp = sqrt(3) / 2;
            wlp = 2 * pi * 10;
            Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), dataf.Ts_log, 'tustin');
        
            % Preallocate cells for 2 axes
            n_axes  = 2;
            obj.T  = cell(1, n_axes);
            obj.P= cell(1, n_axes);
            obj.C = cell(1, n_axes);
            obj.T_gy = cell(1, n_axes);
            obj.Coh_T = cell(1, n_axes);
            obj.Coh_P = cell(1, n_axes);
            obj.Coh_C = cell(1, n_axes);
            
            sinarg_full = dataf.data(:, dataf.ind.sinarg);  % Copy Data to adjust it

            modeFlags = uint32(dataf.data(:, dataf.ind.flightModeFlags));
            bit1 = bitand(modeFlags, uint32(2)) ~= 0;
            angle_active = bit1;

            for ind_axis = 1:n_axes
        
                ind_eval = get_ind_eval( ...
                    dataf.data(:,dataf.ind.sinarg), ...
                    dataf.data(:,dataf.ind.gyroADC(ind_axis)));

                % Only use data recorded while Angle mode was active
                ind_eval = ind_eval & angle_active;
                
                sinarg_ax = sinarg_full;
                sinarg_ax(~ind_eval) = 0;
        
                % Input signal Target Angle
                w = dataf.data(:, dataf.ind.angleTarget(ind_axis));
                inp = apply_rotfiltfilt(Glp, sinarg_ax, w);
        
                % Output signal (Current Angle)
                y = dataf.data(:, dataf.ind.currentAngle(ind_axis));
                out_y = apply_rotfiltfilt(Glp, sinarg_ax, y);

                % Estimation of the Systemtransferfunction
                [T_ax, C_T_ax] = estimate_frequency_response(inp(ind_eval), ...
                    out_y(ind_eval), window, Noverlap, obj.Nest, dataf.Ts_log);

                % Calculation of the plant
                v = dataf.data(:, dataf.ind.gyroADC(ind_axis));
                out_v = apply_rotfiltfilt(Glp, sinarg_ax, v);

                [G_wv_ax, C_G_wv_ax] = estimate_frequency_response(inp(ind_eval), ...
                    out_v(ind_eval), window, Noverlap, obj.Nest, dataf.Ts_log);

                P_angle_ax = T_ax / G_wv_ax;

                % Calculation of the controller
                c = dataf.data(:, dataf.ind.setpoint(ind_axis));
                out_c = apply_rotfiltfilt(Glp, sinarg_ax, c);

                [G_wc_ax,C_G_wc_ax] = estimate_frequency_response(inp(ind_eval), ...
                    out_c(ind_eval), window, Noverlap, obj.Nest, dataf.Ts_log);
    
                Cp_ax = G_wc_ax / (1 - T_ax);
                
                % Transferfunction of the Gyro loop
                T_gy_ax = G_wv_ax / G_wc_ax;


        
                % ----- Store for this axis -----
                obj.T{ind_axis} = T_ax;
                obj.P{ind_axis} = P_angle_ax;
                obj.C{ind_axis} = Cp_ax;
                obj.T_gy{ind_axis} = T_gy_ax;
                obj.Coh_T{ind_axis} = C_T_ax;
                obj.Coh_P{ind_axis} = C_T_ax * C_G_wv_ax;
                obj.Coh_C{ind_axis} = C_T_ax * C_G_wc_ax;
                obj.omega_bode    = 2*pi*P_angle_ax.Frequency;

            end
       end

       function obj = calculate_new_controller(obj, ind_ax, P_new, default_parameters)

            dataf = obj.data_flight;
            
            % Old PT3 not changeable
            angle_lpf_hz = 50;     % frequency of Angle lpf (PT3) 
            
        
            if default_parameters
                P_new = dataf.para.levelPID(1);
            end
        
            fprintf('   used P parameter Angle Control are:\n');
            fprintf('      P: %d\n', dataf.para.levelPID(1));
            fprintf('   new P parameter Angle Control are:\n');
            fprintf('      P: %d\n', P_new);

            C_P_Angle = P_new * 0.1;    % Scaled P

            f = obj.T{ind_ax}.Frequency*2*pi;            % Frequency

            C_P_Angle_frd = frd(C_P_Angle * ones(size(f)), f, dataf.Ts_cntr);
           
            Gf_ana = get_filter('pt3', angle_lpf_hz, dataf.Ts_cntr);  % should be discrete
            
            C_Angle_new = C_P_Angle_frd * Gf_ana;
            obj.C_ana_new = downsample_frd(C_Angle_new, dataf.Ts_log, obj.T{ind_ax}.Frequency);

            P_old = dataf.para.levelPID(1);
            C_P_Angle_old = P_old * 0.1;    % Scaled P

            f = obj.T{ind_ax}.Frequency*2*pi;            % Frequency

            C_P_Angle_frd_old = frd(C_P_Angle_old * ones(size(f)), f, dataf.Ts_cntr);
           
            Gf_ana_old = get_filter('pt3', angle_lpf_hz, dataf.Ts_cntr);  % should be discrete
            
            C_Angle_ana = C_P_Angle_frd_old * Gf_ana_old;
            obj.C_ana = downsample_frd(C_Angle_ana, dataf.Ts_log, obj.T{ind_ax}.Frequency);

            obj.ind_ax = ind_ax;
  
        end


        function obj = get_tuning_data(obj)

            dataf = obj.data_flight;
            gyro = obj.gyro_tuning;
            
            obj.CL_ana = calculate_closed_loop_angle(obj.C_ana, ...
                gyro.T{obj.ind_ax}, obj.P{obj.ind_ax});
            
            obj.CL_ana_new = calculate_closed_loop_angle(obj.C_ana_new, ...
                gyro.T{obj.ind_ax}, obj.P{obj.ind_ax});
        
            f_max = 500;
        
            % Time vector (consistent with Nest)
            T_mean = 0.1 * [-1, 1] + (obj.Nest * dataf.Ts_log) / 2;
            obj.step_time = (0:obj.Nest-1).' * dataf.Ts_log;
        
            % Step responses
            step_resp = [calculate_step_response_from_frd(obj.CL_ana.T, f_max), ...    % Measured Transferfunction
                        calculate_step_response_from_frd(obj.CL_ana_new.T, f_max), ...    % Old parameters
                        calculate_step_response_from_frd(obj.T{obj.ind_ax}, f_max)];  % New parameters
        
            % Normalize around mean window
            idx_mean = (obj.step_time > T_mean(1)) & (obj.step_time < T_mean(2));
            step_resp_mean = mean(step_resp(idx_mean,:), 1);
            obj.step_resp_tra = step_resp ./ step_resp_mean;

            % --- Disturbance Rejection Analysis ---
            % Calculate responses to disturbance inputs
            obj.step_resp_com = [calculate_step_response_from_frd(obj.CL_ana.SP    , f_max), ...    % Used parameters
                         calculate_step_response_from_frd(obj.CL_ana_new.SP, f_max)];       % New parameters
            
            % Center responses around their mean
            step_resp_mean = mean(obj.step_resp_com(obj.step_time > T_mean(1) & obj.step_time < T_mean(2),:));
            obj.step_resp_com = obj.step_resp_com - step_resp_mean;



        end
    end
    
end