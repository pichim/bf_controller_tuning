clc, clear variables
addpath ../../bf_function_libary/
%%

% [fList, pList] = matlab.codetools.requiredFilesAndProducts('bf_controller_tuning_backup.m')
% 165+49+49 = 263 CHF
% 263*1.05  = 276 EUR

% ToDo:
% - axisSum contains feed-forward, this could to be considered in the
%   algorithms
% - the iterm_relax parameter can be used to decide if we need to
%   compensate it's effects
% - create something like para.blackbox_high_resolution in blackbox, so
%   that it can automatically be evaluated if it was a chirp excitaion
% - check out "flightModeFlags" for sinarg evaluation


% choose an axis: 1: roll, 2: pitch, 3: yaw
ind_ax = 3;


% parameters
do_compensate_iterm   = false;
do_show_dev_figures   = false;
do_show_spec_figures  = true;
do_show_motor_figures = false;

do_insert_legends     = false;
multp_fig_nr = ind_ax;

linewidth = 1.2;
set(0, 'defaultAxesColorOrder', get_my_colors);
pos_bode = [0.1514, 0.5838-0.2, 0.7536, 0.3472+0.2; ... % this is a bit hacky
            0.1514, 0.1100    , 0.7536, 0.1917    ];


% bodeoptions (whatever you like)
opt = bodeoptions('cstprefs');
% opt = bodeoptions;
% opt.FreqUnits = 'Hz';
% opt.MagUnits  = 'abs';
% opt.MagScale  = 'log';
% opt.PhaseWrapping = 'on';
% opt.Grid = 'on';

% define quad and build path to *.bbl.csv file
flight_folder = '20231108';

quad = 'apex5';
path = ['/03_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];
% path = ['/03_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_01.bbl.csv'];

% quad = 'apex5hd';
% path = ['/04_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];

% quad = 'flipmini';
% path = ['/05_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];

% quad = 'aosmini';
% path = ['/01_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];


file_path = ['../../../../00_quads', path];


% extract header information
[para, Nheader, ind] = extract_header_information(file_path);


% read the data
tic
try
   load([file_path(1:end-8), '.mat'])
catch exception
   data = readmatrix(file_path, 'NumHeaderLines', Nheader);
   % import_data = importdata(file_path, ',', Nheader);
   % data = import_data.data;
   save([file_path(1:end-8), '.mat'], 'data');
end
[Ndata, Nsig] = size(data) %#ok
toc


% expand index
ind.axisSumPI = ind.axisError(end) + (1:3);
ind.sinarg = ind.debug(1);

% convert and evaluate time
time = (data(:,ind.time) - data(1,ind.time)) * 1.0e-6;
dtime_meas_mus = diff(time) * 1.0e6;

figure(99)
plot(time(1:end-1), dtime_meas_mus), grid on
title(sprintf('Mean: %0.2f mus, Median: %0.2f mus, Std: %0.2f mus\n', ...
      mean(dtime_meas_mus), ...
      median(dtime_meas_mus), ...
      std(dtime_meas_mus)))
xlabel('Time (sec)'), ylabel('Ts log (mus)')
xlim([0, time(end)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% unscale highResolutionGain
if para.blackbox_high_resolution
    blackbox_high_resolution_scale = 10.0;
    ind_bb_high_res = [ind.gyroADC, ind.gyroUnfilt, ind.rcCommand, ind.setpoint(1:3)];
    data(:, ind_bb_high_res) = 1.0 / blackbox_high_resolution_scale * data(:, ind_bb_high_res);
end

% unscale and remap sinarg
sinargScale = 5.0e3;
data(:,ind.sinarg) = 1.0 / sinargScale * data(:,ind.sinarg);

% assign negative sign for pid error
data(:,ind.axisError) = -data(:,ind.axisError);

% create an additional entry for the pi sum
data = [data, data(:,ind.axisP) + data(:,ind.axisI)];


% create different sampling times
Ts      = para.looptime * 1.0e-6;             % gyro
Ts_cntr = para.pid_process_denom * Ts;        % cntrl
Ts_log  = para.frameIntervalPDenom * Ts_cntr; % logging


% get evaluation index
ind_eval = get_ind_eval(data(:,ind.sinarg), data(:,ind.gyroADC(ind_ax)));
data(~ind_eval,ind.sinarg) = 0.0;
T_eval_tot = size(data(ind_eval,ind.sinarg), 1) * Ts_log %#ok


% calculate average throttle
throttle_avg = median(data(ind_eval,ind.setpoint(4))) / 1.0e3;


%% show gyro to select Teval and spectras (gyro and pid sum)

figure(1)
ax(1) = subplot(311);
plot(ax(1), time, data(:,[ind.setpoint(1), ind.gyroUnfilt(1), ind.gyroADC(1)])), grid on, ylabel('Roll (deg/sec)')
title('Gyro Signals')
if do_insert_legends, legend('setpoint', 'gyro', 'gyroADC', 'location', 'best'), end %#ok
ax(2) = subplot(312);
plot(ax(2), time, data(:,[ind.setpoint(2), ind.gyroUnfilt(2), ind.gyroADC(2)])), grid on, ylabel('Pitch (deg/sec)')
ax(3) = subplot(313);
plot(ax(3), time, data(:,[ind.setpoint(3), ind.gyroUnfilt(3), ind.gyroADC(3)])), grid on, ylabel('Yaw (deg/sec)'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax, xlim([0, time(end)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% select data for spectras
data_for_spectras = data(:, [ind.gyroUnfilt, ...
                             ind.gyroADC, ...
                             ind.axisSum, ...
                             ind.setpoint(1:3)]);

Nest     = round(5.0 / Ts_log);
koverlap = 0.9;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);
[pxx, freq] = estimate_spectras(data_for_spectras, window, Noverlap, Nest, Ts_log);
spectras = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)


figure(2)
ax(1) = subplot(211);
plot(ax(1), freq, spectras(:, 1:6)), grid on, ylabel('Gyro (deg/sec)'), set(gca, 'YScale', 'log')
title('Power Spectras')
if do_insert_legends, legend('gyro Roll', 'gyro Pitch', 'gyro Yaw', 'gyroADC Roll', 'gyroADC Pitch', 'gyroADC Yaw', 'location', 'best'), end %#ok
ax(2) = subplot(212);
plot(ax(2), freq, spectras(:, 7:9)), grid on, ylabel('AxisSum'), xlabel('Frequency (Hz)'), set(gca, 'YScale', 'log')
if do_insert_legends, legend('axisSum Roll', 'axisSum Pitch', 'axisSum Yaw', 'location', 'best'), end %#ok
linkaxes(ax), clear ax, axis([0 1/2/Ts_log 1e-3 1e1])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%%

% spectrogram
if (do_show_spec_figures)

    % parameters
    Nest     = round(0.2 / Ts_log);
    koverlap = 0.9;
    Noverlap = round(koverlap * Nest);
    window   = hann(Nest);
    Nres     = 100;

    for spectrogram_nr = 1:3
        [pxx, freq, throttle] = estimate_spectrogram(data(:,ind.gyroUnfilt(spectrogram_nr)), ...
                                                     data(:,ind.setpoint(4)) / 10.0, ...
                                                     window, Noverlap, Nest, Nres, Ts_log);
        spectrograms = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)
        

        figure(22)
        subplot(230 + spectrogram_nr)
        qmesh = pcolor(freq, throttle, spectrograms);
        set(qmesh, 'EdgeColor', 'None', 'FaceColor', 'interp');
        xlabel('Frequency (Hz)'), ylabel('Throttle (%)')
        % colorbar()
        colormap('jet')
        set(gca, 'ColorScale','log')
        clim([3e-2 3e0]);
        ylim([0 100])
    end

    for spectrogram_nr = 1:3
        [pxx, freq, throttle] = estimate_spectrogram(data(:,ind.axisSum(spectrogram_nr)), ...
                                                     data(:,ind.setpoint(4)) / 10.0, ...
                                                     window, Noverlap, Nest, Nres, Ts_log);
        spectrograms = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)
        

        figure(22)
        subplot(230 + spectrogram_nr + 3)
        qmesh = pcolor(freq, throttle, spectrograms);
        set(qmesh, 'EdgeColor', 'None', 'FaceColor', 'interp');
        xlabel('Frequency (Hz)'), ylabel('Throttle (%)')
        % colorbar()
        colormap('jet')
        set(gca, 'ColorScale','log')
        clim([3e-2 3e0]);
        ylim([0 100])
    end
end

%%

if (do_show_dev_figures)
    figure(222) %#ok
    ax(1) = subplot(211);
    plot(ax(1), time, data(:,ind.setpoint(1:3))), grid on, ylabel('Setpoint (deg/sec)')
    title('Chirp Excitation Signals')
    if do_insert_legends, legend('setpoint Roll', 'setpoint Pitch', 'setpoint Yaw', 'location', 'best'), end %#ok
    ax(2) = subplot(212);
    plot(ax(2), time, data(:,ind.sinarg)), grid on, ylabel('Sinarg (rad)'), xlabel('Time (sec)')
    linkaxes(ax, 'x'), clear ax, xlim([0, time(end)])
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
end


figure(3)
ax(1) = subplot(411);
plot(ax(1), time, data(:,ind.gyroUnfilt)), grid on, ylabel('Gyro (deg/sec)')
ax(2) = subplot(412);
plot(ax(2), time, data(:,ind.axisSum)), grid on, ylabel('AxisSum')
ax(3) = subplot(413);
plot(ax(3), time, data(:,ind.motor)), grid on, ylabel('Motor')
ax(4) = subplot(414);
plot(ax(4), time, data(:,ind.setpoint(4))), grid on, ylabel('Throttle'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax, xlim([0, time(end)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%% frequency response estimation and calculation

Nest     = round(2.5 / Ts_log);
koverlap = 0.9;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

% rotating filter
Dlp = sqrt(3) / 2;
wlp = 2 * pi * 10;
Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), Ts_log, 'tustin');


% T  , Gyw: w -> y
inp = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.setpoint(ind_ax)));
out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.gyroADC(ind_ax)) );
[T, C_T] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, Ts_log);


% SCw, Guw: w -> u
out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.axisSum(ind_ax)));
[Guw, C_Guw] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, Ts_log);


%      Gvw: w -> v (v := u only from PI cntrl)
out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.axisSumPI(ind_ax)));
[Gvw, C_Gvw] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, Ts_log);


% P  , Gyu: u -> y
P = T / Guw;


% % P  , Gyu: u -> y (direct measurement, results are slightly worse)
% inp = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.axisSum(ind_ax)));
% out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.gyroADC(ind_ax)));
% [Pd, C_Pd] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, Ts_log);


% controller frf estimates
Cpi = Gvw / (1 - T);
Cd  = Guw * Gvw / T * (1 / Guw - 1 / Gvw);


% index and frequency for bode plots
ind_freq = P.Frequency <= 1/2/Ts_log;
omega_bode = 2*pi*P.Frequency(ind_freq);


if (do_show_dev_figures)
    figure(30) %#ok
    ax(1) = subplot('Position', pos_bode(1,:));
    opt.YLimMode = {'auto'}; opt.MagScale = 'log';
    bode(ax(1), T, 'b', omega_bode, opt), title('Tracking T')
    ax(2) = subplot('Position', pos_bode(2,:));
    opt.MagScale = 'linear';
    bodemag(ax(2), C_T, 'b', omega_bode, opt), title(''), ylabel('Coherence')
    linkaxes(ax, 'x'), clear ax
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

    figure(31)
    ax(1) = subplot('Position', pos_bode(1,:));
    opt.YLimMode = {'auto'}; opt.MagScale = 'log';
    bode(ax(1), Guw, 'g', omega_bode, opt), title('Guw')
    ax(2) = subplot('Position', pos_bode(2,:));
    opt.MagScale = 'linear';
    bodemag(ax(2), C_Guw, 'g', omega_bode, opt), grid on, title(''), ylabel('Coherence')
    linkaxes(ax, 'x'), clear ax
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

    figure(32)
    ax(1) = subplot('Position', pos_bode(1,:));
    opt.YLimMode = {'auto'}; opt.MagScale = 'log';
    bode(ax(1), Gvw, 'r', omega_bode, opt), title('Gvw')
    ax(2) = subplot('Position', pos_bode(2,:));
    opt.MagScale = 'linear';
    bodemag(ax(2), C_Gvw, 'r', omega_bode, opt), title(''), ylabel('Coherence')
    linkaxes(ax, 'x'), clear ax
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
end

%%

% clear motors
% P_motor = 50;
% 
% M_mod = c2d(ss(tf(0.09, [1/(2*pi*8) 1])), Ts_log);
% Tt_M = (50) / (2*pi*100) * pi/180;
% M_mod.InputDelay = round(Tt_M / Ts_log);
% 
% damp(feedback(M_mod * P_motor, 1))

%%

if do_show_motor_figures

    P_motor = 30;

    for motor_nr = 1:4
    
        motor_str = num2str(motor_nr);
        
        figure(300)
        subplot(220 + motor_nr)
        hist3([data(:,ind.motor(motor_nr)), ...
              data(:,ind.eRPM(motor_nr)) / para.motor_poles], ...
              [100 100], ...
              'FaceColor', 'interp', 'EdgeColor', 'interp', 'CDataMode', 'auto')
        title(['Motor ', motor_str])
        ylabel('RPM (rps)'), xlabel('Motor'), set(gca, 'zscale', 'log')
        colorbar
        colormap('jet')
        set(gca,'ColorScale','log')
        view(2)
        
        % Pm 
        inp = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.motor(motor_nr)) );
        out = apply_rotfiltfilt(Glp, data(:,ind.sinarg), data(:,ind.eRPM(motor_nr) ) / para.motor_poles);
        [M, C_M] = estimate_frequency_response(inp(ind_eval), out(ind_eval), window, Noverlap, Nest, Ts_log);
        Lm  =        M*P_motor;
        Sm  = 1.0 / (1.0 + Lm);
        Tm  =            Lm*Sm;
        SPm =             M*Sm;
        SCm =       P_motor*Sm;
        motors.(['M_'  , motor_str]) = M;
        motors.(['C_M_', motor_str]) = C_M;
        motors.(['S_'  , motor_str]) = Sm;
        motors.(['T_'  , motor_str]) = Tm;
        motors.(['SP_' , motor_str]) = SPm;
        motors.(['SC_' , motor_str]) = SCm;
    end
    
    figure(301)
    ax(1) = subplot('Position', pos_bode(1,:));
    opt.YLim = {[1e-4 1e0], [-180 180]}; opt.MagScale = 'log';
    bode(ax(1), motors.M_1, motors.M_2, motors.M_3, motors.M_4, omega_bode, opt), grid on
    title('Motor Pm')
    ax(2) = subplot('Position', pos_bode(2,:));
    opt.YLimMode = {'auto'}; opt.MagScale = 'linear';
    bodemag(ax(2), motors.C_M_1, motors.C_M_2, motors.C_M_3, motors.C_M_4, omega_bode, opt), ylabel('Coherence')
    title('')
    if do_insert_legends, legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'location', 'best'), end %#ok
    linkaxes(ax, 'x'), clear ax
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
    
    
    figure(302)
    ax(1) = subplot(221);
    opt.YLim = {[1e-3 1e1], [-180 180]};
    opt.MagScale = 'log';
    bode(ax(1), motors.T_1, motors.T_2, motors.T_3, motors.T_4, omega_bode, opt), title('Tracking T' )
    ax(2) = subplot(222);
    bode(ax(2), motors.S_1, motors.S_2, motors.S_3, motors.S_4, omega_bode, opt), title('Sensitivity S' )
    ax(3) = subplot(223);
    opt.YLim = {[1e0 1e3], [-180 180]};
    bode(ax(3), motors.SC_1, motors.SC_2, motors.SC_3, motors.SC_4, omega_bode, opt), title('Controller Effort SC')
    ax(4) = subplot(224);
    opt.YLim = {[1e-4 1e-1], [-180 180]};
    bode(ax(4), motors.SP_1, motors.SP_2, motors.SP_3, motors.SP_4, omega_bode, opt), title('Compliance SP')
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


    % step responses
    f_max = 300; % needs to be set according to the coherence, if inf then it has no effect
    step_time = (0:Nest-1).'*Ts_log;
    step_resp = [calculate_step_response_from_frd(motors.M_1, f_max), ...
                 calculate_step_response_from_frd(motors.M_2, f_max), ...
                 calculate_step_response_from_frd(motors.M_3, f_max), ...
                 calculate_step_response_from_frd(motors.M_4, f_max), ...
                 calculate_step_response_from_frd(motors.T_1, f_max), ...
                 calculate_step_response_from_frd(motors.T_2, f_max), ...
                 calculate_step_response_from_frd(motors.T_3, f_max), ...
                 calculate_step_response_from_frd(motors.T_4, f_max)];
    
    figure(303)
    ax(1) = subplot(211);
    plot(ax(1), step_time, step_resp(:,1:4)), grid on    
    title('Motor'), ylabel('RPM (Hz)')
    if do_insert_legends, legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'location', 'best'), end %#ok
    ylim([0 0.1])
    ax(2) = subplot(212);
    plot(ax(2), step_time, step_resp(:,5:8)), grid on
    title('Tracking T'), ylabel('RPM (Hz)')
    ylim([0 1.0])
    linkaxes(ax, 'x'), clear ax, xlim([0 0.5])
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
end


%% downsample analytical controller transferfunction and convert to frd objects

[Cpi_ana, Cd_ana, Gf_ana, PID, para_used] = ...
    calculate_transfer_functions(para, ind_ax, throttle_avg, Ts_cntr);

% downsample analytical controller transferfunction and convert to frd objects
if Gf_ana.Ts < Ts_log % by using Gf_ana.Ts we secure that we do this only once
    Gf_ana  = downsample_frd(Gf_ana , Ts_log, P.Frequency);
    Cpi_ana = downsample_frd(Cpi_ana, Ts_log, P.Frequency);
    Cd_ana  = downsample_frd(Cd_ana , Ts_log, P.Frequency);
end


%% plant and used controllers

figure(expand_multiple_figure_nr(4, multp_fig_nr))
ax(1) = subplot('Position', pos_bode(1,:));
opt.YLim = {[1e-4 1e2], [-180 180]}; opt.MagScale = 'log';
bode(ax(1), P / Gf_ana, 'k', omega_bode, opt), title('Plant P')
hold off, grid on
ax(2) = subplot('Position', pos_bode(2,:));
opt.YLimMode = {'auto'}; opt.MagScale = 'linear';
bodemag(ax(2), C_T * C_Guw, 'k', omega_bode, opt), title(''), ylabel('Coherence')
linkaxes(ax, 'x'), clear ax
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

% compare analytical to estimated controllers
figure(expand_multiple_figure_nr(8, multp_fig_nr))
opt.YLim = {[1e-2 1e2], [-180 180]}; opt.MagScale = 'log';
bode(Cpi, Cd, Cpi_ana, Cd_ana, omega_bode, opt), title('Cpi, Cd')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%% new controller and filter parameters

tic

pid_axis = {'rollPID', 'pitchPID', 'yawPID'};

% PID parameters
fprintf('   used PID parameters are:\n');
fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], ...
    para.(pid_axis{ind_ax})(1:3));

% inform user about parameters
para_used_fieldnames = fieldnames(para_used);
Npara_used = size(para_used_fieldnames, 1);
fprintf('   used parameters are:\n');
for i = 1:Npara_used
    fprintf(['      ', para_used_fieldnames{i},': %d\n'], eval(['round(', 'para_used.', para_used_fieldnames{i}, ');']));
end

% copy parameters (in case you dont change anything)
para_new = para;

% you can use the following command to generate the text
% get_switch_case_text_from_para(para)

switch quad
    case 'apex5'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 600;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 160;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 240;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.10, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        switch ind_ax
            case 1 % roll: [50, 75, 38, 38, 0]
                P_new       = 55;
                I_ratio_new = 75/75;
                D_new       = 32+2;
            case 2 % pitch: [62, 92, 46, 46, 0]
                P_new       = 68;
                I_ratio_new = 96/92;
                D_new       = 39+2;
            case 3 % yaw: [43, 85, 3, 3, 0]
                P_new       = 43;
                I_ratio_new = 85/85;
                D_new       = 3;
            otherwise
        end
        % % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        % para_new.gyro_lpf            = 0;       % dono what this is
        % para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        % para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        % para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        % para_new.gyro_lowpass2_hz    = 600;     % frequency of gyro lpf 2
        % para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        % para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        % para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        % para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        % para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        % para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        % para_new.dterm_lpf2_hz       = 160;     % frequency of dterm lpf 2
        % para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        % para_new.dterm_notch_hz      = 235;     % frequency of dterm notch
        % para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.10, para_new.dterm_notch_hz); % damping of dterm notch
        % para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        % switch ind_ax
        %     case 1 % roll: [40, 75, 13, 0, 0]
        %         P_new       = 40;
        %         I_ratio_new = 75/75;
        %         D_new       = 13;
        %     case 2 % pitch: [52, 92, 21, 0, 0]
        %         P_new       = 52;
        %         I_ratio_new = 92/92;
        %         D_new       = 21;
        %     case 3 % yaw: [33, 85, 0, 0, 0]
        %         P_new       = 33;
        %         I_ratio_new = 85/85;
        %         D_new       = 0;
        %     otherwise
        % end
     case 'apex5hd'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 600;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 160;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 235;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.10, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        para_new.gyro_llc_freq_hz    = 100;     % frequency of gyro llc
        para_new.gyro_llc_phase      = 0;       % phase of gyro llc
        para_new.dterm_llc_freq_hz   = 100;     % frequency of dterm llc
        para_new.dterm_llc_phase     = 0;       % phase of dterm llc
        para_new.pterm_llc_freq_hz   = 100;     % frequency of pterm llc
        para_new.pterm_llc_phase     = 0;       % phase of pterm llc
        switch ind_ax
            case 1 % roll: [50, 82, 35, 35, 0]
                P_new       = 50;
                I_ratio_new = 82/82;
                D_new       = 35;
            case 2 % pitch: [62, 88, 42, 42, 0]
                P_new       = 62;
                I_ratio_new = 88/88;
                D_new       = 42;
            case 3 % yaw: [32, 78, 0, 0, 0]
                P_new       = 32;
                I_ratio_new = 78/78;
                D_new       = 0;
            otherwise
        end
    case 'flipmini'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 600;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 160;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 0;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.00, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        para_new.gyro_llc_freq_hz    = 100;     % frequency of gyro llc
        para_new.gyro_llc_phase      = 0;       % phase of gyro llc
        para_new.dterm_llc_freq_hz   = 100;     % frequency of dterm llc
        para_new.dterm_llc_phase     = 0;       % phase of dterm llc
        para_new.pterm_llc_freq_hz   = 100;     % frequency of pterm llc
        para_new.pterm_llc_phase     = 0;       % phase of pterm llc
        switch ind_ax
            case 1 % roll: [46, 66, 32, 32, 0]
                P_new       = 46;
                I_ratio_new = 66/66;
                D_new       = 32;
            case 2 % pitch: [71, 105, 49, 49, 0]
                P_new       = 71;
                I_ratio_new = 105/105;
                D_new       = 49;
            case 3 % yaw: [30, 65, 0, 0, 0]
                P_new       = 30;
                I_ratio_new = 65/65;
                D_new       = 0;
            otherwise
        end
    case 'aosmini'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 600;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 160;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 0;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.00, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        para_new.gyro_llc_freq_hz    = 100;     % frequency of gyro llc
        para_new.gyro_llc_phase      = 0;       % phase of gyro llc
        para_new.dterm_llc_freq_hz   = 100;     % frequency of dterm llc
        para_new.dterm_llc_phase     = 0;       % phase of dterm llc
        para_new.pterm_llc_freq_hz   = 100;     % frequency of pterm llc
        para_new.pterm_llc_phase     = 0;       % phase of pterm llc
        switch ind_ax
            case 1 % roll: [39, 60, 27, 27, 0]
                P_new       = 39;
                I_ratio_new = 60/60;
                D_new       = 27;
            case 2 % pitch: [56, 81, 40, 40, 0]
                P_new       = 56;
                I_ratio_new = 81/81;
                D_new       = 40;
            case 3 % yaw: [30, 44, 0, 0, 0]
                P_new       = 30;
                I_ratio_new = 44/44;
                D_new       = 0;
            otherwise
        end
    otherwise
        warning(' no valid quad selected');
end

% scale to new PID parameters
pid_scale = [get_pid_scale(ind_ax), 1];
PID_new(1) = P_new * pid_scale(1);
fI         = PID(2) / (2 * pi * PID(1)); % extract fn from initial parametrization
fI_new     = fI * I_ratio_new;
PID_new(2) = 2 * pi * PID_new(1) * fI_new;
PID_new(3) = D_new * pid_scale(3);
PID_new(4) = 0;

fprintf('   used fI is: %0.2f Hz\n\n', fI);

% new PID parameters
fprintf('   new used PID parameters are:\n');
para_new.(pid_axis{ind_ax}) = round( PID_new ./ pid_scale);
para_new.(pid_axis{ind_ax}) = [para_new.(pid_axis{ind_ax})(1:3), ...
                               para_new.(pid_axis{ind_ax})(3), ...
                               para_new.(pid_axis{ind_ax})(4)];
fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], ...
    para_new.(pid_axis{ind_ax})(1:3));

[Cpi_ana_new, Cd_ana_new, Gf_ana_new, PID_new, para_used_new] = ...
    calculate_transfer_functions(para_new, ind_ax, throttle_avg, Ts_cntr);


% inform user about new parameters
para_used_fieldnames_new = fieldnames(para_used_new);
Npara_used_new = size(para_used_fieldnames_new, 1);
fprintf('   new used parameters are:\n');
for i = 1:Npara_used_new
    fprintf(['      ', para_used_fieldnames_new{i},': %d\n'], ...
        eval(['round(', 'para_used_new.', para_used_fieldnames_new{i}, ');']));
end

fprintf('   new used fI is: %0.2f Hz\n\n', fI_new);


% downsample analytical controller transferfunction and convert to frd objects
if Gf_ana_new.Ts < Ts_log % by using Gf_ana.Ts we secure that we do this only once
    Gf_ana_new  = downsample_frd(Gf_ana_new , Ts_log, P.Frequency);
    Cpi_ana_new = downsample_frd(Cpi_ana_new, Ts_log, P.Frequency);
    Cd_ana_new  = downsample_frd(Cd_ana_new , Ts_log, P.Frequency);
end


CL_ana     = calculate_closed_loop(Cpi_ana    , tf(1,1,Ts_log), P / Gf_ana, Gf_ana    , Cd_ana    );
CL_ana_new = calculate_closed_loop(Cpi_ana_new, tf(1,1,Ts_log), P / Gf_ana, Gf_ana_new, Cd_ana_new);
if do_compensate_iterm
    % compensate only PI part
    Cpi_com = Cpi / Cpi_ana; %#ok
    CL_ana_      = calculate_closed_loop(Cpi_ana     * Cpi_com, tf(1,1,Ts_log), P / Gf_ana, Gf_ana    , Cd_ana    );
    CL_ana_new_  = calculate_closed_loop(Cpi_ana_new * Cpi_com, tf(1,1,Ts_log), P / Gf_ana, Gf_ana_new, Cd_ana_new);
    CL_ana.T     = CL_ana_.T;
    CL_ana_new.T = CL_ana_new_.T;
end


% closed-loop
figure(expand_multiple_figure_nr(5, multp_fig_nr))
ax(1) = subplot(221);
opt.YLim = {[1e-3 1e1], [-180 180]}; opt.MagScale = 'log';
bodemag(ax(1), CL_ana.T , CL_ana_new.T , T, omega_bode, opt), title('Tracking T')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end %#ok
ax(2) = subplot(222);
bodemag(ax(2), CL_ana.S , CL_ana_new.S , omega_bode, opt), title('Sensitivity S')
ax(3) = subplot(223);
opt.YLim = {[1e-2 1e2], [-180 180]};
bodemag(ax(3), CL_ana.SC, CL_ana_new.SC, omega_bode, opt), title('Controller Effort SC')
ax(4) = subplot(224);
opt.YLim = {[1e-4 1e0], [-180 180]};
bodemag(ax(4), CL_ana.SP, CL_ana_new.SP, omega_bode, opt), title('Compliance SP')
linkaxes(ax, 'x'), clear ax
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% step responses
f_max = para.dyn_notch_min_hz; % needs to be set according to the coherence, if inf then it has no effect
Tmean = 0.1 * [-1, 1] + (Nest * Ts_log) / 2;
step_time = (0:Nest-1).'*Ts_log;

step_resp = [calculate_step_response_from_frd(CL_ana.T    , f_max), ...
             calculate_step_response_from_frd(CL_ana_new.T, f_max), ...
             calculate_step_response_from_frd(T           , f_max)];
step_resp_mean = mean(step_resp(step_time > Tmean(1) & step_time < Tmean(2),:));
step_resp = step_resp ./ step_resp_mean;

figure(expand_multiple_figure_nr(6, multp_fig_nr))
ax(1) = subplot(211);
plot(ax(1), step_time, step_resp), grid on, ylabel('Gyro (deg/sec)')
title('Tracking T')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end %#ok
ylim([0 1.3])

step_resp = [calculate_step_response_from_frd(CL_ana.SP    , f_max), ...
             calculate_step_response_from_frd(CL_ana_new.SP, f_max)];
step_resp_mean = mean(step_resp(step_time > Tmean(1) & step_time < Tmean(2),:));
step_resp = step_resp - step_resp_mean;

ax(2) = subplot(212);
plot(ax(2), step_time, step_resp), grid on
title('Compliance SP'), xlabel('Time (sec)'), ylabel('Gyro (deg/sec)')
ylim([-0.2 1.1])
linkaxes(ax, 'x'), clear ax, xlim([0 0.5])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% controllers
figure(expand_multiple_figure_nr(7, multp_fig_nr))
opt.YLim = {[1e-1 1e2], [-180 180]};
bode(CL_ana.C, CL_ana_new.C, omega_bode, opt)
title('Controller C')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end %#ok
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% group and phase delay tracking
freq  = CL_ana.T.Frequency;
phase       = unwrap(angle(squeeze(CL_ana.T.ResponseData)));
group_delay = -gradient(phase, 2*pi*freq);
phase_delay = -phase ./ (2*pi*freq);
phase_new       = unwrap(angle(squeeze(CL_ana_new.T.ResponseData)));
group_delay_new = -gradient(phase_new, 2*pi*freq);
phase_delay_new = -phase_new ./ (2*pi*freq);

figure(expand_multiple_figure_nr(8, multp_fig_nr))
subplot(121)
semilogx(freq(ind_freq), [group_delay(ind_freq), group_delay_new(ind_freq)] * 1e3), grid on
xlim([0 f_max]), ylim([0 30])
ylabel('Group Delay (ms)'), xlabel('Frequency (Hz)'), title('Group Delay T')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end %#ok
subplot(122)
semilogx(freq(ind_freq), [phase_delay(ind_freq), phase_delay_new(ind_freq)] * 1e3), grid on
xlim([0 f_max]), ylim([0 30])
xlabel('Frequency (Hz)'), ylabel('Phase Delay (ms)'), title('Phase Delay Tracking T')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end %#ok
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% bode open-loop L
figure(expand_multiple_figure_nr(9, multp_fig_nr))
opt.YLim = {[1e-2 1e2], [-180 180]}; opt.MagScale = 'log';
bode(CL_ana.L, CL_ana_new.L, omega_bode, opt), title('Open-Loop L')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end %#ok
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

toc
% Elapsed time is 15.670831 seconds.