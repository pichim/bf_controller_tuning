%==========================================================================
% FLIGHT ANALYZER - Betaflight Controller Analysis ANALYSIS CLASS
%==========================================================================
% Purpose: 
%   Get Spectra and Spectogram out of Data
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

classdef flight_analyzer
    properties       
        % Raw flight data inputs

        time             % Time vector [s]
        data             % Flgiht data
        ind              % Columns
        Ts_log           % Logging Time [s]

        % Used in different functions
        freq_spectra
        spectra
        freq_spectogram
        spectrogram_unf
        spectrogram_fil
        throttle_all

        % Default values
        linewidth            (1,1) double  = 1.2
        
        
    end

    methods
        function obj = flight_analyzer(data, ind, Ts_log)
            obj.data = data;
            obj.ind = ind;
            obj.Ts_log = Ts_log;

        end

        function obj = calculate_spectra(obj, Nestfaspec, koverlapspec)
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
                
            
            % Select data columns for spectral analysis
            data_for_spectra = obj.data(:,[obj.ind.gyroUnfilt, ...
                                       obj.ind.gyroADC, ...
                                       obj.ind.axisSum, ...
                                       obj.ind.setpoint(1:3)]);
            
            % Configure spectral estimation parameters
            Nestspec = round(Nestfaspec / obj.Ts_log);    % Window length in samples
            windowspec   = hann(Nestspec, 'periodic');        % Hann window for better frequency resolution
            Noverlap = floor(koverlapspec * Nestspec);    % Overlap for smoother estimates

            % Calculate power spectral density
            [pxx, obj.freq_spectra] = estimate_spectra(data_for_spectra, windowspec, Noverlap, Nestspec, obj.Ts_log);
            obj.spectra = sqrt(pxx); % Convert power to amplitude spectra (dc needs to be scaled differently)


        end
        function obj = calculate_spectogram(obj, Nestfaspec, koverlapspec)

            Nestspec = round(Nestfaspec / obj.Ts_log);    % Window length in samples
            windowspec   = hann(Nestspec, 'periodic');        % Hann window for better frequency resolution
            Noverlap = floor(koverlapspec * Nestspec);    % Overlap for smoother estimates

            % Calculate throttle resolution
            Nres = floor(max(obj.data(:,obj.ind.setpoint(4))) / 1e1 / 2); % should give 40 at 80% throttle constrain

            % Initialize storage for multiple spectrograms
            num_spectrograms = 3;    % One per axis
            obj.spectrogram_unf = cell(1, num_spectrograms);
            obj.spectrogram_fil = cell(1, num_spectrograms);
            obj.freq_spectogram = cell(1, num_spectrograms);
            obj.throttle_all = cell(1, num_spectrograms);

            % Calculate spectrograms for unfiltered gyro data
            for spectrogram_nr = 1:num_spectrograms
                [pxx, freq, throttle] = estimate_spectrogram( ...
                    obj.data(:, obj.ind.gyroUnfilt(spectrogram_nr)), ...    % Raw gyro data
                    obj.data(:, obj.ind.setpoint(4)) / 10.0, ...            % Throttle values
                    windowspec, Noverlap, Nestspec, Nres, obj.Ts_log);

                obj.spectrogram_unf{spectrogram_nr} = sqrt(pxx); % Convert power to amplitude
                obj.freq_spectogram{spectrogram_nr} = freq;
                obj.throttle_all{spectrogram_nr} = throttle;
            end

                 % Calculate spectrograms for filtered gyro data
            for spectrogram_nr = 1:num_spectrograms
                [pxx, freq, throttle] = estimate_spectrogram( ...
                    obj.data(:, obj.ind.gyroADC(spectrogram_nr)), ...    % Filtered gyro data
                    obj.data(:, obj.ind.setpoint(4)) / 10.0, ...         % Throttle values
                    windowspec, Noverlap, Nestspec, Nres, obj.Ts_log);

                obj.spectrogram_fil{spectrogram_nr} = sqrt(pxx); % Convert power to amplitude
                obj.freq_spectogram{spectrogram_nr} = freq;
                obj.throttle_all{spectrogram_nr} = throttle;
            end
            
        end
       
    end

end




