%==========================================================================
% FLIGHT DATA - Betaflight Controller Analysis DATA IMPORT CLASS
%==========================================================================
% Purpose: 
%   Read Data for further calculations
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

classdef flight_data
    properties
        % File handling
        file_path        % Path to the flight log file (.bbl.csv)
        
        % Raw flight data
        time             % Time vector [s]
        data             % Flgiht data
        ind              % Columns
        para             % Parameters old
        
        % Calculated data
        Ts_log           % Logging Time [s]
        Ts_cntr          

        % Default values
        linewidth            (1,1) double  = 1.2
        
    end

    methods

        function obj = flight_data(file_path)
            obj.file_path = file_path;
        end

         function obj = get_data(obj)
            % --- Load and Process Flight Log Data ---
            [obj.para, Nheader, obj.ind, ind_cntr] = extract_header_information(obj.file_path);
            
            % Load data from CSV or cached MAT file for faster processing
            tic
            try
               load_data = load([obj.file_path(1:end-8), '.mat']);
               obj.data = load_data.data;
            catch exception
               data = readmatrix(obj.file_path, 'NumHeaderLines', Nheader);
               obj.data = data;
               save([obj.file_path(1:end-8), '.mat'], 'data');
            end
            toc
            
            % --- Data Preprocessing ---
            % Expand obj.indices for additional data columns
            % obj.ind.axisSumPI = ind_cntr + (1:3);
            
            % Create an additional entry for the PI sum
            axisSumPI_data = obj.data(:,obj.ind.axisP) + obj.data(:,obj.ind.axisI);
            obj.data = [obj.data, axisSumPI_data];

            % Now assign the new indices from actual data size
            obj.ind.axisSumPI = size(obj.data,2)-2 : size(obj.data,2);

            obj.ind.sinarg = obj.ind.debug(1);
            % Current version
            % obj.ind.currentAngle = [obj.ind.debug(2), obj.ind.debug(5)];
            % obj.ind.angleTarget = [obj.ind.debug(3), obj.ind.debug(6)];
            % obj.ind.angleRate = [obj.ind.debug(4), obj.ind.debug(7)];
            
            % New Verion in the future
            obj.ind.currentAngle = [obj.ind.debug(5), obj.ind.debug(7)];
            obj.ind.angleTarget = [obj.ind.debug(6), obj.ind.debug(8)];
            
            % Convert microseconds to seconds for time vector
            obj.time = (obj.data(:,obj.ind.time) - obj.data(1,obj.ind.time)) * 1.0e-6;
            
            % Unscale highResolutionGain
            if obj.para.blackbox_high_resolution
                blackbox_high_resolution_scale = 10.0;
                ind_bb_high_res = [obj.ind.gyroADC, obj.ind.gyroUnfilt, ...
                    obj.ind.rcCommand, obj.ind.setpoint(1:3), obj.ind.currentAngle(1:2), ...
                    obj.ind.angleTarget(1:2)];
                obj.data(:, ind_bb_high_res) = 1.0 / blackbox_high_resolution_scale * obj.data(:, ind_bb_high_res);
            end
            
            % Unscale and remap sinarg
            sinargScale = 5.0e3;
            obj.data(:,obj.ind.sinarg) = 1.0 / sinargScale * obj.data(:,obj.ind.sinarg);

            % Unscale and remap heading
            obj.data(:,obj.ind.heading(1:3)) = obj.data(:,obj.ind.heading(1:3)) * 100;
            
            % Create different sampling times
            Ts      = obj.para.looptime * 1.0e-6;             % Gyro loop
            obj.Ts_cntr = obj.para.pid_process_denom * Ts;        % Control loop
            obj.Ts_log  = obj.para.frameIntervalPDenom * obj.Ts_cntr; % Logging loop                       
         end        
     end
end
