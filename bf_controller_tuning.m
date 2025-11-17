clc, clear variables
addpath lib/
%%

% [files, products] = matlab.codetools.requiredFilesAndProducts('bf_controller_tuning.m')
% disp(products)


%%

% TODO:
%   BF Controller Tuning:
%   - The iterm_relax parameter can be used to decide if we need to
%     compensate it's effects
%   - Evaluate "flightModeFlags" for sinarg evaluation
%   Betaflight:
%   - Create something like para.blackbox_high_resolution in blackbox, so
%     that it can automatically be evaluated if it was a chirp excitation
%   - Make chirp start (sin or cos) and the amplitude reduction below 1 Hz
%     as a setting
%   - Make it so that chirp parameters can be changes via the goggles

% Choose an axis: 1: roll, 2: pitch, 3: yaw
ind_ax = 1;

% -------------------------------------------------------------------------

% % Define quad and path to *.bbl.csv file
% flight_folder = '20250907';

% quad = 'aosmini';
% log_name = '20250907_aosmini_00.bbl.csv';

% quad = 'apex5';
% log_name = '20250907_apex5_00.bbl.csv';

% quad = 'flipmini';
% log_name = '20250907_flipmini_00.bbl.csv';

% -------------------------------------------------------------------------

% % Define quad and path to *.bbl.csv file
% flight_folder = '20250908';

% quad = 'flipmini';
% log_name = '20250908_flipmini_00.bbl.csv';

% -------------------------------------------------------------------------

% % Define quad and path to *.bbl.csv file
% flight_folder = '20250918';
% 
% quad = 'aosmini';
% log_name = '20250918_aosmini_00.bbl.csv';

% -------------------------------------------------------------------------

% Define quad and path to *.bbl.csv file
flight_folder = '20251104';

quad = 'yurisdrone';
log_name = 'LOG000.TXT.csv';

% -------------------------------------------------------------------------

file_path = fullfile(flight_folder, log_name);

% Evaluation parameters
do_compensate_iterm  = true;
do_show_spec_figures = true;
do_insert_legends    = false;

multp_fig_nr = ind_ax;

% Defines
set(cstprefs.tbxprefs, 'MagnitudeUnits', 'abs');
set(cstprefs.tbxprefs, 'FrequencyUnits', 'Hz');
set(cstprefs.tbxprefs, 'UnwrapPhase', 'Off');
set(cstprefs.tbxprefs, 'Grid', 'On');

linewidth = 1.2;
set(0, 'defaultAxesColorOrder', get_my_colors);
pos_bode = [0.1514, 0.5838-0.2, 0.7536, 0.3472+0.2; ... % this is a bit hacky
            0.1514, 0.1100    , 0.7536, 0.1917    ];

% Bodeoptions
opt = bodeoptions('cstprefs');

% Extract header information
[para, Nheader, ind, ind_cntr] = extract_header_information(file_path);

% Read the data
%  - If its the first time from the .csv and save a mat, otherwise the
%    .mat. This increases load speed significantly.
tic
try
   load([file_path(1:end-8), '.mat'])
catch exception
   data = readmatrix(file_path, 'NumHeaderLines', Nheader);
   save([file_path(1:end-8), '.mat'], 'data');
end
[Ndata, Nsig] = size(data)
toc

% Expand index
ind.axisSumPI = ind_cntr + (1:3);
ind.sinarg = ind.debug(1);

% Convert and evaluate time
time = (data(:,ind.time) - data(1,ind.time)) * 1.0e-6;
delta_time_mus = diff(time) * 1.0e6;

figure(99)
plot(time(1:end-1), delta_time_mus), grid on
title(sprintf('Mean: %0.2f mus, Median: %0.2f mus, Std: %0.2f mus\n', ...
      mean(delta_time_mus), ...
      median(delta_time_mus), ...
      std(delta_time_mus)))
xlabel('Time (sec)'), ylabel('Ts log (mus)')
xlim([0, time(end)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

% Unscale highResolutionGain
if para.blackbox_high_resolution
    blackbox_high_resolution_scale = 10.0;
    ind_bb_high_res = [ind.gyroADC, ind.gyroUnfilt, ind.rcCommand, ind.setpoint(1:3)];
    data(:, ind_bb_high_res) = 1.0 / blackbox_high_resolution_scale * data(:, ind_bb_high_res);
end

% Unscale and remap sinarg
sinargScale = 5.0e3;
data(:,ind.sinarg) = 1.0 / sinargScale * data(:,ind.sinarg);

% Assign negative sign for pid error
data(:,ind.axisError) = -data(:,ind.axisError);

% Create an additional entry for the pi sum
data = [data, data(:,ind.axisP) + data(:,ind.axisI)];

% Create different sampling times
Ts      = para.looptime * 1.0e-6;             % Gyro loop
Ts_cntr = para.pid_process_denom * Ts;        % Control loop
Ts_log  = para.frameIntervalPDenom * Ts_cntr; % Logging loop

% Get evaluation index where Chirp was active
ind_eval = get_ind_eval(data(:,ind.sinarg), data(:,ind.gyroADC(ind_ax)));
data(~ind_eval,ind.sinarg) = 0.0;
T_eval_tot = size(data(ind_eval,ind.sinarg), 1) * Ts_log

% Calculate average throttle
throttle_avg = median(data(ind_eval,ind.setpoint(4))) / 1.0e3;


%% show Gyro to select Teval and spectra (gyro and pid sum)

figure(1)
ax(1) = subplot(311);
plot(ax(1), time, data(:,[ind.setpoint(1), ind.gyroUnfilt(1), ind.gyroADC(1)])), grid on, ylabel('Roll (deg/sec)')
title('Gyro Signals')
if do_insert_legends, legend('setpoint', 'gyro', 'gyroADC', 'location', 'best'), end
ax(2) = subplot(312);
plot(ax(2), time, data(:,[ind.setpoint(2), ind.gyroUnfilt(2), ind.gyroADC(2)])), grid on, ylabel('Pitch (deg/sec)')
ax(3) = subplot(313);
plot(ax(3), time, data(:,[ind.setpoint(3), ind.gyroUnfilt(3), ind.gyroADC(3)])), grid on, ylabel('Yaw (deg/sec)'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax, xlim([0, time(end)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

% Select data for spectra
data_for_spectra = data(:,[ind.gyroUnfilt, ...
                           ind.gyroADC, ...
                           ind.axisSum, ...
                           ind.setpoint(1:3)]);

% Parameters
Nest     = round(2.0 / Ts_log);
koverlap = 0.9;
Noverlap = floor(koverlap * Nest);
window   = hann(Nest, 'periodic');
[pxx, freq] = estimate_spectra(data_for_spectra, window, Noverlap, Nest, Ts_log);
spectra = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)

figure(2)
ax(1) = subplot(211);
plot(ax(1), freq, spectra(:, 1:6)), grid on, ylabel('Gyro (deg/sec)'), set(gca, 'YScale', 'log')
title('Magnitude Spectra')
if do_insert_legends, legend('gyro Roll', 'gyro Pitch', 'gyro Yaw', 'gyroADC Roll', 'gyroADC Pitch', 'gyroADC Yaw', 'location', 'best'), end
ax(2) = subplot(212);
plot(ax(2), freq, spectra(:, 7:9)), grid on, ylabel('AxisSum'), xlabel('Frequency (Hz)'), set(gca, 'YScale', 'log')
if do_insert_legends, legend('axisSum Roll', 'axisSum Pitch', 'axisSum Yaw', 'location', 'best'), end
linkaxes(ax), clear ax, axis([0 1/2/Ts_log 1e-3 1e1])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%%

% Spectrogram
if (do_show_spec_figures)

    % Parameters
    Nest     = round(0.2 / Ts_log);
    koverlap = 0.9;
    Noverlap = floor(koverlap * Nest);
    window   = hann(Nest, 'periodic');
    Nres     = floor(max(data(:,ind.setpoint(4))) / 1e1 / 2) % should give 40 at 80% throttle constrain

    c_lim = [5e-2 3e0];

    for spectrogram_nr = 1:3
        [pxx, freq, throttle] = estimate_spectrogram(data(:,ind.gyroUnfilt(spectrogram_nr)), ...
                                                     data(:,ind.setpoint(4)) / 10.0, ...
                                                     window, Noverlap, Nest, Nres, Ts_log);
        spectrograms = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)

        figure(22)
        subplot(230 + spectrogram_nr)
        qmesh = pcolor(freq, throttle, spectrograms);
        set(qmesh, 'EdgeColor', 'None');
        % xlabel('Frequency (Hz)')
        if spectrogram_nr == 1
            ylabel('Throttle (%)')
        end
        % colorbar()
        colormap('jet')
        set(gca, 'ColorScale', 'log')
        clim(c_lim);
        ylim([0 100])
    end

    for spectrogram_nr = 1:3
        [pxx, freq, throttle] = estimate_spectrogram(data(:,ind.gyroADC(spectrogram_nr)), ...
                                                     data(:,ind.setpoint(4)) / 10.0, ...
                                                     window, Noverlap, Nest, Nres, Ts_log);
        spectrograms = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)

        figure(22)
        subplot(230 + spectrogram_nr + 3)
        qmesh = pcolor(freq, throttle, spectrograms);
        set(qmesh, 'EdgeColor', 'None');
        xlabel('Frequency (Hz)')
        if spectrogram_nr == 1
            ylabel('Throttle (%)')
        end
        % colorbar()
        colormap('jet')
        set(gca, 'ColorScale', 'log')
        clim(c_lim);
        ylim([0 100])
    end
end


%% Some relevant fligth data

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


%% Frequency response estimation and calculation

% Parameters
Nest     = round(2.5 / Ts_log);
koverlap = 0.9;
Noverlap = floor(koverlap * Nest);
window   = hann(Nest, 'periodic');

% Linear filter for zero phase excitation filter (apply_rotfiltfilt)
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

% Calculated controller frequency response estimates
Cpi = Gvw / (1 - T);
Cd  = Guw * Gvw / T * (1 / Guw - 1 / Gvw);

% Index and frequency for bode plots
omega_bode = 2*pi*P.Frequency;


%% Downsample analytical controller transferfunction and convert to frd objects

[Cpi_ana, Cd_ana, Gf_ana, PID, para_used] = ...
    calculate_transfer_functions(para, ind_ax, throttle_avg, Ts_cntr);

if Gf_ana.Ts < Ts_log % by using Gf_ana.Ts we secure that we do this only once
    Gf_ana  = downsample_frd(Gf_ana , Ts_log, P.Frequency);
    Cpi_ana = downsample_frd(Cpi_ana, Ts_log, P.Frequency);
    Cd_ana  = downsample_frd(Cd_ana , Ts_log, P.Frequency);
end


%% Plant and used controllers

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

% Compare analytical to estimated controllers
figure(expand_multiple_figure_nr(5, multp_fig_nr))
opt.YLim = {[1e-2 1e2], [-180 180]}; opt.MagScale = 'log';
bode(Cpi, Cd, Cpi_ana, Cd_ana, omega_bode, opt), title('Cpi, Cd')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%% New controller and filter parameters

tic

pid_axis = {'rollPID', 'pitchPID', 'yawPID'};

% PID parameters
fprintf('   used PID parameters are:\n');
fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], ...
    para.(pid_axis{ind_ax})(1:3));

% Inform user about parameters
para_used_fieldnames = fieldnames(para_used);
Npara_used = size(para_used_fieldnames, 1);
fprintf('   used parameters are:\n');
for i = 1:Npara_used
    fprintf(['      ', para_used_fieldnames{i},': %d\n'], eval(['round(', 'para_used.', para_used_fieldnames{i}, ');']));
end

% First create new parameters the same as the actual ones
para_new = para;

% You can use the following command to generate the text below for the
% actual parameters
% get_switch_case_text_from_para(para)

switch quad
    case 'aosmini'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 800;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 120;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 0;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.00, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        switch ind_ax
            case 1 % roll: [33, 52, 26, 0]
                P_new       = 33;
                I_ratio_new = 52/52;
                D_new       = 26;
            case 2 % pitch: [58, 98, 44, 0]
                P_new       = 58;
                I_ratio_new = 98/98;
                D_new       = 44;
            case 3 % yaw: [42, 65, 3, 0]
                P_new       = 42;
                I_ratio_new = 65/65;
                D_new       = 3;
        end
    case 'apex5'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 800;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 520]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.15], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 130;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 235;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.15, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        switch ind_ax
            case 1 % roll: [49, 83, 33, 0]
                P_new       = 49;
                I_ratio_new = 83/83;
                D_new       = 33;
            case 2 % pitch: [61, 103, 39, 0]
                P_new       = 61;
                I_ratio_new = 103/103;
                D_new       = 39;
            case 3 % yaw: [42, 104, 3, 0]
                P_new       = 42;
                I_ratio_new = 104/104;
                D_new       = 3;
        end
    case 'flipmini'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 800;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 140;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 0;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.00, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        switch ind_ax
            case 1 % roll: [46, 74, 30, 0]
                P_new       = 46;
                I_ratio_new = 74/74;
                D_new       = 30;
            case 2 % pitch: [71, 118, 47, 0]
                P_new       = 71;
                I_ratio_new = 118/118;
                D_new       = 47;
            case 3 % yaw: [35, 70, 3, 0]
                P_new       = 35;
                I_ratio_new = 70/70;
                D_new       = 3;
        end
    case 'yurisdrone'
        % type: 0: PT1, 1: BIQUAD, 2: PT2, 3: PT3
        para_new.gyro_lpf            = 0;       % dono what this is
        para_new.gyro_lowpass_hz     = 0;       % frequency of gyro lpf 1
        para_new.gyro_soft_type      = 0;       % type of gyro lpf 1
        para_new.gyro_lowpass_dyn_hz = [0, 0];  % dyn gyro lpf overwrites gyro_lowpass_hz
        para_new.gyro_lowpass2_hz    = 800;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 150];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 100;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 3;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 0;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.00, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        switch ind_ax
            case 1 % roll: [45, 80, 30, 40]
                P_new       = 45;
                I_ratio_new = 80/80;
                D_new       = 30;
            case 2 % pitch: [47, 84, 34, 46]
                P_new       = 47;
                I_ratio_new = 84/84;
                D_new       = 34;
            case 3 % yaw: [45, 80, 0, 0]
                P_new       = 45;
                I_ratio_new = 80/80;
                D_new       = 0;
        end
    otherwise
        warning(' no valid quad selected');
end

% Scale to new PID parameters
pid_scale = [get_pid_scale(ind_ax), 1];
PID_new(1) = P_new * pid_scale(1);
fI         = PID(2) / (2 * pi * PID(1)); % extract fn from initial parametrization
fI_new     = fI * I_ratio_new;
PID_new(2) = 2 * pi * PID_new(1) * fI_new;
PID_new(3) = D_new * pid_scale(3);
PID_new(4) = 0;

fprintf('   used fI is: %0.2f Hz\n\n', fI);

% New PID parameters
fprintf('   new PID parameters are:\n');
para_new.(pid_axis{ind_ax}) = round( PID_new ./ pid_scale);
para_new.(pid_axis{ind_ax}) = [para_new.(pid_axis{ind_ax})(1:3), ...
                               para_new.(pid_axis{ind_ax})(3), ...
                               para_new.(pid_axis{ind_ax})(4)];
fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], ...
    para_new.(pid_axis{ind_ax})(1:3));

[Cpi_ana_new, Cd_ana_new, Gf_ana_new, PID_new, para_used_new] = ...
    calculate_transfer_functions(para_new, ind_ax, throttle_avg, Ts_cntr);

% Inform user about new parameters
para_used_fieldnames_new = fieldnames(para_used_new);
Npara_used_new = size(para_used_fieldnames_new, 1);
fprintf('   new parameters are:\n');
for i = 1:Npara_used_new
    fprintf(['      ', para_used_fieldnames_new{i},': %d\n'], ...
        eval(['round(', 'para_used_new.', para_used_fieldnames_new{i}, ');']));
end

fprintf('   new used fI is: %0.2f Hz\n\n', fI_new);

% Downsample analytical controller transferfunction and convert to frd objects
if Gf_ana_new.Ts < Ts_log % by using Gf_ana.Ts we secure that we do this only once
    Gf_ana_new  = downsample_frd(Gf_ana_new , Ts_log, P.Frequency);
    Cpi_ana_new = downsample_frd(Cpi_ana_new, Ts_log, P.Frequency);
    Cd_ana_new  = downsample_frd(Cd_ana_new , Ts_log, P.Frequency);
end

CL_ana     = calculate_closed_loop(Cpi_ana    , tf(1,1,Ts_log), P / Gf_ana, Gf_ana    , Cd_ana    );
CL_ana_new = calculate_closed_loop(Cpi_ana_new, tf(1,1,Ts_log), P / Gf_ana, Gf_ana_new, Cd_ana_new);
if do_compensate_iterm
    % Compensate only PI part
    Cpi_com = Cpi / Cpi_ana;
    CL_ana_      = calculate_closed_loop(Cpi_ana     * Cpi_com, tf(1,1,Ts_log), P / Gf_ana, Gf_ana    , Cd_ana    );
    CL_ana_new_  = calculate_closed_loop(Cpi_ana_new * Cpi_com, tf(1,1,Ts_log), P / Gf_ana, Gf_ana_new, Cd_ana_new);
    CL_ana.T     = CL_ana_.T;
    CL_ana_new.T = CL_ana_new_.T;
end

% Closed-loop bode plots (gang of four)
figure(expand_multiple_figure_nr(6, multp_fig_nr))
ax(1) = subplot(221);
opt.YLim = {[1e-3 1e1], [-180 180]}; opt.MagScale = 'log';
bodemag(ax(1), CL_ana.T , CL_ana_new.T , T, omega_bode, opt), title('Tracking T')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end
ax(2) = subplot(222);
bodemag(ax(2), CL_ana.S , CL_ana_new.S , omega_bode, opt), title('Sensitivity S')
ax(3) = subplot(223);
opt.YLim = {[1e-2 1e2], [-180 180]};
bodemag(ax(3), CL_ana.SC, CL_ana_new.SC, omega_bode, opt), title('Controller Effort SC')
ax(4) = subplot(224);
opt.YLim = {[1e-3 1e1], [-180 180]};
bodemag(ax(4), CL_ana.SP, CL_ana_new.SP, omega_bode, opt), title('Compliance SP')
linkaxes(ax, 'x'), clear ax
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

% Step responses
f_max = min([para.dyn_notch_min_hz, para.gyro_rpm_notch_min]);
T_mean = 0.1 * [-1, 1] + (Nest * Ts_log) / 2;
step_time = (0:Nest-1).'*Ts_log;

% Actual controller parameters
step_resp = [calculate_step_response_from_frd(CL_ana.T    , f_max), ...
             calculate_step_response_from_frd(CL_ana_new.T, f_max), ...
             calculate_step_response_from_frd(T           , f_max)];
step_resp_mean = mean(step_resp(step_time > T_mean(1) & step_time < T_mean(2),:));
step_resp = step_resp ./ step_resp_mean;

figure(expand_multiple_figure_nr(7, multp_fig_nr))
ax(1) = subplot(211);
plot(ax(1), step_time, step_resp), grid on, ylabel('Gyro (deg/sec)')
title('Tracking T')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end
ylim([0 1.3])

% New controller parameters
step_resp = [calculate_step_response_from_frd(CL_ana.SP    , f_max), ...
             calculate_step_response_from_frd(CL_ana_new.SP, f_max)];
step_resp_mean = mean(step_resp(step_time > T_mean(1) & step_time < T_mean(2),:));
step_resp = step_resp - step_resp_mean;

ax(2) = subplot(212);
plot(ax(2), step_time, step_resp), grid on
title('Compliance SP'), xlabel('Time (sec)'), ylabel('Gyro (deg/sec)')
ylim([-0.2 1.1])
linkaxes(ax, 'x'), clear ax, xlim([0 0.5])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

% Controllers
figure(expand_multiple_figure_nr(8, multp_fig_nr))
opt.YLim = {[1e-1 1e2], [-180 180]};
bode(CL_ana.C, CL_ana_new.C, omega_bode, opt)
title('Controller C')
if do_insert_legends, legend('actual', 'new', 'location', 'best'), end
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

toc
