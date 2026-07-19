%==========================================================================
% MAIN - Betaflight Controller Analysis File
%==========================================================================
% Purpose: 
%   Tool for analysing flight logs and tuning PID controllers for a quadcopter
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

%% General

% Initialize workspace
clc, clear variables, close all
addpath(genpath('lib'));
addpath logs/
addpath class/

% Show Legends
do_insert_legends = true;

% Angle Tuning Required 
angle_tuning_required = true;

% Create Plotter Class
plotter = plot_utils(do_insert_legends);

%% Loading Data

% =========================================================================
%  Define path to *.bbl.csv file for the 
% =========================================================================

log_folder = 'logs';
flight_folder = 'example_logs';
log_name = 'Gyro_Angle.csv';
file_path = fullfile(log_folder, flight_folder, log_name);

df = flight_data(file_path);
% Load Data
df = df.get_data();


roll = 1; pitch = 2; yaw = 3;
group1 = {
    struct( ...
        'idx', [df.ind.setpoint(roll), df.ind.gyroUnfilt(roll), df.ind.gyroADC(roll)], ...
        'ylabel', 'Roll Rate [deg/s]' )

    struct( ...
        'idx', [df.ind.setpoint(pitch), df.ind.gyroUnfilt(pitch), df.ind.gyroADC(pitch)], ...
        'ylabel', 'Pitch Rate [deg/s]' )

    struct( ...
        'idx', [df.ind.setpoint(yaw), df.ind.gyroUnfilt(yaw), df.ind.gyroADC(yaw)], ...
        'ylabel', 'Yaw Rate [deg/s]' )
};

plotter.plotFlightData(df, group1,'Flight Gyro Data');

 group2 = {
        struct('idx', df.ind.gyroUnfilt(1:3), ...
               'ylabel', 'Gyro (deg/s)', ...
               'legend', ["Roll","Pitch","Yaw"])

        struct('idx', df.ind.axisSum(1:3), ...
               'ylabel', 'AxisSum', ...
               'legend', ["Roll","Pitch","Yaw"])

        struct('idx', df.ind.motor, ...
               'ylabel', 'Motors', ...
               'legend', ["Motor 1","Motor 2","Motor 3","Motor 4"])

        struct('idx', df.ind.setpoint(4), ...
               'ylabel', 'Throttle', ...
               'legend', "setpoint")
    };

plotter.plotFlightData(df, group2,'Flight Overview');

 group3 = {
        struct('idx', [df.ind.currentAngle(1:2), df.ind.heading(1:2)], ...
               'ylabel', 'Current Angle [°]', ...
               'legend', ["Roll Current","Pitch Current","Roll Heading","Pitch Heading"])

        struct('idx', df.ind.angleTarget(1:2), ...
               'ylabel', 'Target Angle [°]', ...
               'legend', ["Roll","Pitch"])

    };

plotter.plotFlightData(df, group3,'Target Overview');
plotter.plot_Eval_Time(df.time);

%% Get Bodeplots

gyro_tuning = gyro_ctrl_tuning(df);


resolution_factor_tuning = 2.5;    % Window length for spectral analysis (seconds)
overlap_tuning = 0.9;              % Overlap factor for spectral analysis (0-1)
gyro_tuning = gyro_tuning.calculate_transfer_func(resolution_factor_tuning, overlap_tuning);

% Options (gyro_tuning/angle_tuning, roll/pitch/yaw, 'Gyro'/'Angle', ...
% 'Plant'/'Complementary Sensitivity'/'Controller')

plotter.plot_Bode_Plant(gyro_tuning, roll, 'Gyro', 'Plant');


%% Flight Analyser
analysis_flight = flight_analyzer(df.data, df.ind, df.Ts_log);

% Data for Spectra
resolution_factor_spectra = 0.2;    % Window length for spectral analysis (seconds)
overlap_spectra = 0.9;              % Overlap factor for spectral analysis (0-1)
analysis_flight = analysis_flight.calculate_spectra(resolution_factor_spectra, ...
    overlap_spectra);

% Data for Spectogram
resolution_factor_spectogram = 0.2;    % Window length for spectral analysis (seconds)
overlap_spectogram = 0.9;              % Overlap factor for spectral analysis (0-1)
analysis_flight = analysis_flight.calculate_spectogram(resolution_factor_spectogram, ...
    overlap_spectogram);

%% Plot Flight Analyser
plotter.plot_Spectogram(analysis_flight, 3);
plotter.plot_Gyro_spectra(analysis_flight);

%% Gyro Tuning Data

% =========================================================================
%  ¨ Selection: 1: roll, 2: pitch, 3: yaw
% =========================================================================

ind_ax = 1;     % keep it now until plot_utils is finished

% I-term Relax on/off
do_compensate_iterm = true;

% New and old parameters are the same
default_parameters_gyro = false; 

% =========================================================================
%  First flight: Parameters
% =========================================================================

% Configure filters to match Betaflight settings
% All frequencies are in Hz
% Filter types:
%   0: PT1 (First order lowpass)
%   1: BIQUAD (Second order)
%   2: PT2 (Second order lowpass) 
%   3: PT3 (Third order lowpass)
para_new.gyro_lpf            = 0;       % if lpf is static
para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
para_new.gyro_lowpass2_hz    = 800;     % frequency of gyro lpf 2
para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
para_new.gyro_notch_hz       = [0, 0];  % frequency of gyro notch 1 and 2

% Gyro notch cutoff: either computed from D and center frequency or set manually
% para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.0, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 
para_new.gyro_notch_cutoff   = [0, 0]; % % Cutoff frequency gyro notch 1 and 2

para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
para_new.dterm_lpf2_hz       = 140;     % frequency of dterm lpf 2
para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
para_new.dterm_notch_hz      = 0;       % frequency of dterm notch

% D Term notch cutoff:either computed from D and center frequency or set manually
% para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.00, para_new.dterm_notch_hz); % damping of dterm notch
para_new.dterm_notch_cutoff  = 0;     % Cutoff frequency dterm notch
para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)

%--------------------------------------------------------------------------
%  tune your PIDs here
%--------------------------------------------------------------------------
% Adjust multipliers PID values to tune the response
% Higher P: more immediate response but possible oscillations
% Higher I: better steady-state tracking but slower response
% Higher D: better damping but noise sensitive

switch ind_ax
    case 1 % Roll PID values [default: 45, 80, 30]
        P_new       =35;%46
        I_new       = 70;%74
        D_new       = 22;%30
    case 2 % Pitch PID values [default: 47, 84, 34]
        P_new       = 49;
        I_new       = 96;
        D_new       = 35;
    case 3 % Yaw PID values [default: 45, 80, 0]
        P_new       = 45;
        I_new       = 86;
        D_new       = 1;
end

gyro_tuning = gyro_tuning.calculate_new_controller(ind_ax, P_new, I_new, D_new, ...
    default_parameters_gyro, para_new);
gyro_tuning = gyro_tuning.get_tuning_data(do_compensate_iterm);

% plotter.plot_Bode_Contr(ind_ax, do_insert_legends);
plotter.plot_Gang_of_Four(gyro_tuning,  'Gyro');
plotter.plot_Step_Compliance(gyro_tuning,  'Gyro', 'Gyro (deg/sec)');

%% Angle Tuning Data
modeFlags = df.data(:, df.ind.flightModeFlags);
angle_tuning_possible = any(modeFlags == 8388675);

if angle_tuning_possible && angle_tuning_required && ind_ax ~= yaw

    angle_tuning = angle_ctrl_tuning(df,gyro_tuning);
    angle_tuning = angle_tuning.calculate_Angle_trans(resolution_factor_tuning, overlap_tuning);

    plotter.plot_Bode_Plant(angle_tuning, roll, 'Angle', 'Plant');

    default_parameters_angle = false;

    % PT3 Angle Control
    P_Angle = 120;

    angle_tuning = angle_tuning.calculate_new_controller(ind_ax, P_Angle, ...
        default_parameters_angle);
    angle_tuning = angle_tuning.get_tuning_data();
    
    plotter.plot_Gang_of_Four(angle_tuning,  'Angle');
    plotter.plot_Step_Response(angle_tuning,  'Angle', 'Angle (deg)');
end