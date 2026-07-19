%==========================================================================
% MAIN_CLASS - Betaflight Flight Controller Analysis and Tuning Class
%==========================================================================
% Purpose: Processes flight logs and tunes PID controllers for quadcopters
%
% Main functionalities:
% - Flight log data processing
% - Spectral analysis of gyro signals
% - Transfer function estimation
% - PID controller tuning
% - Step response analysis
%
% Author: [Your Name]
% Date: [Current Date]
%==========================================================================

classdef gyro_ctrl_tuning

    properties
        % File handling
        file_path        % Path to the flight log file (.bbl.csv)
        
        % Controller parameters
        para_new         % New/modified controller parameters
        ind_ax           % Selected axis (1:roll, 2:pitch, 3:yaw)
        do_compensate_iterm % Enable I-term compensation flag
        P_new            % New proportional gain
        I_new      % New integral gain ratio
        D_new            % New derivative gain
        
        % Raw flight data
        unfgyroData      % Unfiltered gyro data [deg/s]
        gyroData         % Filtered gyro data [deg/s]
        setpoint         % Controller setpoints
        motorData        % Motor output values
        time             % Time vector [s]
        
        % Spectral analysis results
        unfgyroSpec      % Unfiltered gyro spectra
        adcgyroSpec      % ADC (filtered) gyro spectra
        axisSumSpec      % Axis sum spectra
        Specfreq         % Frequency vector for spectra [Hz]
        Ts_log           % Logging sample time [s]
        axisSumData      % Sum of P and I terms
        
        % Transfer function data
        transfData       % Plant transfer function
        transfCoher      % Transfer function coherence
        transfOmega      % Frequency vector for transfer functions [rad/s]
        transfCpi        % PI controller transfer function
        transfCD         % D controller transfer function
        transfCpiAna     % Analytical PI controller
        transfCDAna      % Analytical D controller
        CloLoAan         % Closed loop analysis (current)
        CloLoAanNew      % Closed loop analysis (new parameters)
        transfT          % Complementary sensitivity function
        
        % Step response analysis
        step_resp_tra    % Step response trajectories
        step_resp_com    % Step response comparison
        step_resp_tim    % Time vector for step response
        
        % Spectrogram data
        SpecAmplUnf      % Unfiltered spectrogram amplitudes
        SpecfreqUnf      % Frequencies for unfiltered spectrograms
        SpecthroUnf      % Throttle values for unfiltered spectrograms
        SpecAmplFil      % Filtered spectrogram amplitudes
        SpecfreqFil      % Frequencies for filtered spectrograms
        SpecthroFil      % Throttle values for filtered spectrograms
        num_spectrograms % Number of spectrograms to generate
        
        % Analysis parameters
        Nestfaspec       % Window length for spectral analysis
        koverlapspec     % Overlap factor for spectral analysis
        Nestfatra        % Window length for transfer function estimation
        koverlaptra      % Overlap factor for transfer function estimation
        default_parameters (1,1) logical = false % Use default parameters flag
    end

    methods
        % =================================================================
        %  Constructor
        % =================================================================
        % MAIN_CLASS Constructor for flight data analysis class
        function obj = gyro_ctrl_tuning(file_path, para_new, ind_ax, do_compensate_iterm, ...
                P_new, I_ratio_new, D_new, Nestfaspec,koverlapspec,Nestfatra, koverlaptra, ...
                default_parameters)
            
            % Initialize object properties
            obj.file_path = file_path;
            obj.para_new = para_new;
            obj.ind_ax = ind_ax;
            obj.do_compensate_iterm = do_compensate_iterm;
            obj.P_new = P_new;
            obj.I_new = I_ratio_new;
            obj.D_new = D_new;
            obj.Nestfaspec = Nestfaspec;
            obj.koverlapspec = koverlapspec;
            obj.Nestfatra = Nestfatra;
            obj.koverlaptra = koverlaptra;
            obj.default_parameters = default_parameters;
        end

        % =================================================================
        %  Main Analysis
        % =================================================================
        % RUN Execute complete flight data analysis pipeline
        %
        % This method:
        % 1. Loads and preprocesses flight log data
        % 2. Performs spectral analysis of gyro signals
        % 3. Calculates transfer functions and frequency responses
        % 4. Computes step responses for controller evaluation
        % 5. Generates new controller parameters if requested
        function obj = run(obj)
            % --- Load and Process Flight Log Data ---
            [para, Nheader, ind, ind_cntr] = extract_header_information(obj.file_path);
            
            % Load data from CSV or cached MAT file for faster processing
            tic
            try
               load([obj.file_path(1:end-8), '.mat'])
            catch exception
               data = readmatrix(obj.file_path, 'NumHeaderLines', Nheader);
               save([obj.file_path(1:end-8), '.mat'], 'data');
            end
            toc
            
            % --- Data Preprocessing ---
            % Expand indices for additional data columns
            ind.axisSumPI = ind_cntr + (1:3);
            ind.sinarg = ind.debug(1);
            
            % Convert microseconds to seconds for time vector
            obj.time = (data(:,ind.time) - data(1,ind.time)) * 1.0e-6;
            
            % Unscale highResolutionGain
            if para.blackbox_high_resolution
                blackbox_high_resolution_scale = 10.0;
                ind_bb_high_res = [ind.gyroADC, ind.gyroUnfilt, ind.rcCommand, ind.setpoint(1:3)];
                data(:, ind_bb_high_res) = 1.0 / blackbox_high_resolution_scale * data(:, ind_bb_high_res);
            end
            
            % Unscale and remap sinarg
            sinargScale = 5.0e3;
            data(:,ind.sinarg) = 1.0 / sinargScale * data(:,ind.sinarg);
                        
            % Create an additional entry for the pi sum
            data = [data, data(:,ind.axisP) + data(:,ind.axisI)];
            
            % Create different sampling times
            Ts      = para.looptime * 1.0e-6;             % Gyro loop
            Ts_cntr = para.pid_process_denom * Ts;        % Control loop
            obj.Ts_log  = para.frameIntervalPDenom * Ts_cntr; % Logging loop
            
            % Get evaluation index where Chirp was active
            ind_eval = get_ind_eval(data(:,ind.sinarg), data(:,ind.gyroADC(obj.ind_ax)));
            data(~ind_eval,ind.sinarg) = 0.0;
            T_eval_tot = size(data(ind_eval,ind.sinarg), 1) * obj.Ts_log
            
            % Calculate average throttle
            throttle_avg = median(data(ind_eval,ind.setpoint(4))) / 1.0e3;
            
            
            %% 
            % =============================================================
            %  Spectral Analysis
            % =============================================================
            % Performs frequency domain analysis of gyro signals and PID sums
            % 
            % Steps:
            % 1. Configure spectral estimation parameters
            % 2. Calculate power spectra using Hann window
            % 3. Convert power to amplitude spectra
            % 4. Store results for unfiltered, filtered gyro and axis sums
                
            
            obj.setpoint    = data(:, ind.setpoint(1:4));
            obj.unfgyroData = data(:, ind.gyroUnfilt(1:3));
            obj.gyroData    = data(:, ind.gyroADC(1:3));
            obj.motorData = data(:,ind.motor);
            obj.axisSumData = data(:, ind.axisSum);
                    
            % Select data columns for spectral analysis
            data_for_spectra = data(:,[ind.gyroUnfilt, ...
                                       ind.gyroADC, ...
                                       ind.axisSum, ...
                                       ind.setpoint(1:3)]);
            
            % Configure spectral estimation parameters
            Nestspec = round(obj.Nestfaspec / obj.Ts_log);    % Window length in samples
            windowspec   = hann(Nestspec, 'periodic');        % Hann window for better frequency resolution
            Noverlap = floor(obj.koverlapspec * Nestspec);    % Overlap for smoother estimates

            % Calculate power spectral density
            [pxx, freq] = estimate_spectra(data_for_spectra, windowspec, Noverlap, Nestspec, obj.Ts_log);
            spectra = sqrt(pxx); % Convert power to amplitude spectra (dc needs to be scaled differently)

            % Store results
            obj.Specfreq = freq;                 % Frequency vector for all spectra
            obj.unfgyroSpec = spectra(:,1:3);    % Raw gyro spectra
            obj.adcgyroSpec = spectra(:,4:6);    % Filtered gyro spectra
            obj.axisSumSpec = spectra(:,7:9);    % PID sum spectra                     
            
            %%
            % =============================================================
            %  Spectrogram
            % =============================================================
            % Generates time-frequency analysis plots showing how frequency
            % content changes with throttle position
            %
            % Parameters:
            % - Uses same window settings as spectral analysis
            % - Throttle resolution determined by max throttle value
            % - Generates spectrograms for each axis (roll, pitch, yaw)

            % Calculate throttle resolution
            Nres = floor(max(data(:,ind.setpoint(4))) / 1e1 / 2) % should give 40 at 80% throttle constrain

            % Initialize storage for multiple spectrograms
            obj.num_spectrograms = 3;    % One per axis
            spectrograms = cell(1, obj.num_spectrograms);
            freq_all = cell(1, obj.num_spectrograms);
            throttle_all = cell(1, obj.num_spectrograms);
            
            % Calculate spectrograms for unfiltered gyro data
            for spectrogram_nr = 1:obj.num_spectrograms
                [pxx, freq, throttle] = estimate_spectrogram( ...
                    data(:, ind.gyroUnfilt(spectrogram_nr)), ...    % Raw gyro data
                    data(:, ind.setpoint(4)) / 10.0, ...            % Throttle values
                    windowspec, Noverlap, Nestspec, Nres, obj.Ts_log);
            
                spectrograms{spectrogram_nr} = sqrt(pxx); % Convert power to amplitude
                freq_all{spectrogram_nr} = freq;
                throttle_all{spectrogram_nr} = throttle;
            end
            
            % Store unfiltered results
            obj.SpecAmplUnf = spectrograms;
            obj.SpecfreqUnf = freq_all;
            obj.SpecthroUnf = throttle_all;
           
            % Calculate spectrograms for filtered gyro data
            for spectrogram_nr = 1:obj.num_spectrograms
                [pxx, freq, throttle] = estimate_spectrogram( ...
                    data(:, ind.gyroADC(spectrogram_nr)), ...    % Filtered gyro data
                    data(:, ind.setpoint(4)) / 10.0, ...         % Throttle values
                    windowspec, Noverlap, Nestspec, Nres, obj.Ts_log);
            
                spectrograms{spectrogram_nr} = sqrt(pxx); % Convert power to amplitude
                freq_all{spectrogram_nr} = freq;
                throttle_all{spectrogram_nr} = throttle;
            end

            % Store filtered results
            obj.SpecAmplFil = spectrograms;
            obj.SpecfreqFil = freq_all;
            obj.SpecthroFil = throttle_all; 
         
            %% 
            % =============================================================
            %  Frequency response estimation and calculation
            % =============================================================
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

            % Analysis window parameters
            Nest     = round(obj.Nestfatra / obj.Ts_log);    % Window length in samples
            Noverlap = floor(obj.koverlaptra * Nest);        % Overlap between windows
            window   = hann(Nest, 'periodic');               % Hanning window for analysis
            
            % Design linear filter for zero phase excitation filter (apply_rotfiltfilt)
            Dlp = sqrt(3) / 2;    % Damping ratio
            wlp = 2 * pi * 10;    % Cutoff frequency [rad/s]
            Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), ...    % Discrete filter
                    obj.Ts_log, 'tustin');                   % Using Tustin transform
            
            % Calculate complementary sensitivity (T) and input-output responses
            % T  , Gyw: w -> y
            inp = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.setpoint(obj.ind_ax)));
            out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.gyroADC(obj.ind_ax)) );
            [T, C_T] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, obj.Ts_log);
            
            % Calculate control sensitivity (Represents total controller output response)
            % SCw, Guw: w -> u
            out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.axisSum(obj.ind_ax)));
            [Guw, C_Guw] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, obj.Ts_log);
            
            % Calculate PI controller response (v is the output of just the PI portion of controller)
            % Gvw: w -> v
            out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.axisSumPI(obj.ind_ax)));
            [Gvw, C_Gvw] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, obj.Ts_log);
            
            % Calculate plant response (indirect method: P = T/Guw for better noise immunity)
            % P  , Gyu: u -> y
            P = T / Guw;
            
            % % P  , Gyu: u -> y (direct measurement, results are slightly worse)
            % inp = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.axisSum(ind_ax)));
            % out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.gyroADC(ind_ax)));
            % [Pd, C_Pd] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, Ts_log);
            
            % Calculate controller frequency responses
            % Split into PI and D components for analysis
            Cpi = Gvw / (1 - T);    % PI controller portion
            Cd  = Guw * Gvw / T * (1 / Guw - 1 / Gvw);    % D controller portion
            
            % Prepare frequency vector for Bode plots
            omega_bode = 2*pi*P.Frequency;    % Convert Hz to rad/s
            
            % Generate analytical controller transfer functions
            [Cpi_ana, Cd_ana, Gf_ana, PID, para_used] = ...
                calculate_transfer_functions(para, obj.ind_ax, throttle_avg, Ts_cntr);

            % Downsample analytical responses to match logging frequency
            if Gf_ana.Ts < obj.Ts_log % by using Gf_ana.Ts we secure that we do this only once
                Gf_ana  = downsample_frd(Gf_ana , obj.Ts_log, P.Frequency);
                Cpi_ana = downsample_frd(Cpi_ana, obj.Ts_log, P.Frequency);
                Cd_ana  = downsample_frd(Cd_ana , obj.Ts_log, P.Frequency);
            end
            
            % =============================================================
            %  Plant and Controller Transfer Functions
            % =============================================================
            % Stores calculated transfer functions and coherence data for 
            % system analysis and controller design
            %
            % Transfer Functions Stored:
            % - Plant (P/Gf_ana): Actual plant dynamics normalized by filter
            % - Coherence: Data quality metric (product of T and Guw coherence)
            % - Controllers: Current and analytical PI/D controllers
            % - Complementary Sensitivity (T): Closed loop response
            %
            % All frequency responses stored as FRD (Frequency Response Data) objects
            
            % Store plant and coherence data
            obj.transfData = P / Gf_ana;      % Normalized plant dynamics
            obj.transfCoher = C_T * C_Guw;    % Overall coherence metric
            obj.transfOmega = omega_bode;     % Frequency vector [rad/s]

            % Store measured and analytical controller responses
            obj.transfCpi = Cpi;           % Measured PI controller
            obj.transfCD = Cd;             % Measured D controller
            obj.transfCpiAna = Cpi_ana;    % Analytical PI model
            obj.transfCDAna = Cd_ana;      % Analytical D model

            % Store complementary sensitivity
            obj.transfT = T;    % Measured closed-loop response
            
            % =============================================================
            %  New Controller Parameter Calculation
            % =============================================================
            % Calculates and validates new PID controller parameters based on
            % analysis results and user inputs.
            %
            % Process:
            % 1. Load/calculate default or new parameters
            % 2. Scale PID gains based on axis selection
            % 3. Update integral frequency
            % 4. Calculate new transfer functions
            % 5. Generate step responses for validation

            tic
            
            % Initialize axis names for reporting
            pid_axis = {'rollPID', 'pitchPID', 'yawPID'};

            % Handle default parameter case
            if obj.default_parameters
                obj.para_new = para;
                 pid_vec = para.(pid_axis{obj.ind_ax});

                 % Use existing parameters
                 obj.P_new = pid_vec(1);    % Current P gain
                 obj.I_new = pid_vec(2);       % Keep same I gain
                 obj.D_new = pid_vec(3);    % Current D gain
            end
            
            % Display used PID configuration
            fprintf('   used PID parameters are:\n');
            fprintf(['      ', pid_axis{obj.ind_ax}, ':  %d, %d, %d\n'], ...
                para.(pid_axis{obj.ind_ax})(1:3));
            
            % Display all used parameters
            para_used_fieldnames = fieldnames(para_used);
            Npara_used = size(para_used_fieldnames, 1);
            fprintf('   used parameters are:\n');
            for i = 1:Npara_used
                fprintf(['      ', para_used_fieldnames{i},': %d\n'], ...
                    eval(['round(', 'para_used.', para_used_fieldnames{i}, ');']));
            end
            
            % --- Calculate New PID Parameters ---
            % Get axis-specific scaling factors (roll/pitch/yaw have different gains)
            pid_scale = [get_pid_scale(obj.ind_ax), 1];    % Get axis scaling factors

            % Calculate new gains with scaling
            PID_new(1) = obj.P_new * pid_scale(1);         % Scale P gain
            fI         = PID(2) / (2 * pi * PID(1));       % Extract current I frequency
            PID_new(2) = obj.I_new *pid_scale(2);
            fI_new     = PID_new(2) / (2 * pi * PID_new(1));    % Scale I frequency
            PID_new(3) = obj.D_new * pid_scale(3);         % Scale D gain
            PID_new(4) = 0;                                % No feedforward
            
            % Display integral frequencies
            fprintf('   used fI is: %0.2f Hz\n\n', fI);
            
            % --- Update and Display New Parameters ---
            fprintf('   new PID parameters are:\n');

            % Round scaled PID values and store in parameter structure
            obj.para_new.(pid_axis{obj.ind_ax}) = round( PID_new ./ pid_scale);
            obj.para_new.(pid_axis{obj.ind_ax}) = [obj.para_new.(pid_axis{obj.ind_ax})(1:3), ...
                                           obj.para_new.(pid_axis{obj.ind_ax})(3), ...
                                           obj.para_new.(pid_axis{obj.ind_ax})(4)];

            % Display new PID configuration
            fprintf(['      ', pid_axis{obj.ind_ax}, ':  %d, %d, %d\n'], ...
                obj.para_new.(pid_axis{obj.ind_ax})(1:3));

            % --- Generate New Transfer Functions ---
            % Calculate analytical transfer functions with new parameters
            [Cpi_ana_new, Cd_ana_new, Gf_ana_new, PID_new, para_used_new] = ...
                calculate_transfer_functions(obj.para_new, obj.ind_ax, throttle_avg, Ts_cntr);
            
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
            if Gf_ana_new.Ts < obj.Ts_log % by using Gf_ana.Ts we secure that we do this only once
                Gf_ana_new  = downsample_frd(Gf_ana_new , obj.Ts_log, P.Frequency);
                Cpi_ana_new = downsample_frd(Cpi_ana_new, obj.Ts_log, P.Frequency);
                Cd_ana_new  = downsample_frd(Cd_ana_new , obj.Ts_log, P.Frequency);
            end
            
            % --- Calculate Closed Loop Responses ---
            % Generate closed loop responses for both current and new parameters
            CL_ana     = calculate_closed_loop(Cpi_ana    , tf(1,1,obj.Ts_log), P / Gf_ana, Gf_ana    , Cd_ana    );
            CL_ana_new = calculate_closed_loop(Cpi_ana_new, tf(1,1,obj.Ts_log), P / Gf_ana, Gf_ana_new, Cd_ana_new);

            % Apply I-term compensation if enabled
            if obj.do_compensate_iterm
                % Compensate only PI part
                Cpi_com = Cpi / Cpi_ana;

                % Recalculate closed loop responses with compensation
                CL_ana_      = calculate_closed_loop(Cpi_ana     * Cpi_com, tf(1,1,obj.Ts_log), P / Gf_ana, Gf_ana    , Cd_ana    );
                CL_ana_new_  = calculate_closed_loop(Cpi_ana_new * Cpi_com, tf(1,1,obj.Ts_log), P / Gf_ana, Gf_ana_new, Cd_ana_new);

                % Update complementary sensitivity
                CL_ana.T     = CL_ana_.T;
                CL_ana_new.T = CL_ana_new_.T;
            end

            % Store final closed loop responses
            obj.CloLoAan = CL_ana;
            obj.CloLoAanNew = CL_ana_new;
            
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
            f_max = min([para.dyn_notch_min_hz, para.gyro_rpm_notch_min]);

            % Define time window for response analysis
            T_mean = 0.1 * [-1, 1] + (Nest * obj.Ts_log) / 2;    % Analysis window [s]
            step_time = (0:Nest-1).'*obj.Ts_log;                 % Time vector [s]
            
            % --- Tracking Performance Analysis ---
            % Calculate step responses for setpoint tracking
            step_resp = [calculate_step_response_from_frd(CL_ana.T    , f_max), ...    % Used parameters
                         calculate_step_response_from_frd(CL_ana_new.T, f_max), ...    % New parameters
                         calculate_step_response_from_frd(T           , f_max)];       % Measured response

            % Normalize responses around their mean values
            step_resp_mean = mean(step_resp(step_time > T_mean(1) & step_time < T_mean(2),:));
            step_resp = step_resp ./ step_resp_mean;

            % Store tracking response results
            obj.step_resp_tra = step_resp;
            obj.step_resp_tim = step_time;
            
            % --- Disturbance Rejection Analysis ---
            % Calculate responses to disturbance inputs
            step_resp = [calculate_step_response_from_frd(CL_ana.SP    , f_max), ...    % Used parameters
                         calculate_step_response_from_frd(CL_ana_new.SP, f_max)];       % New parameters
            
            % Center responses around their mean
            step_resp_mean = mean(step_resp(step_time > T_mean(1) & step_time < T_mean(2),:));
            step_resp = step_resp - step_resp_mean;

            % Store disturbance response results
            obj.step_resp_com = step_resp;
                       
            toc
        end
    end
end