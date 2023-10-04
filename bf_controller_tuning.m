clc, clear variables
addpath ../../bf_function_libary/
%%

% ToDo:
% - axisSum contains feed-forward, this could to be considered in the
%   algorithms
% - the iterm_relax parameter can be used to decide if we need to
%   compensate it's effects
% - create something like para.blackbox_high_resolution in blackbox, so
%   that it can automatically be evaluated if it was a chirp excitaion
% - check out "flightModeFlags" for sinarg evaluation


% choose an axis: 1: roll, 2: pitch, 3: yaw
ind_ax = 1;


% parameters
do_compensate_iterm = false;
do_show_dev_figures = false;
linewidth = 1.2;
multp_fig_nr = ind_ax;


% bodeoptions (whatever you like)
opt = bodeoptions('cstprefs');
% opt = bodeoptions;
% opt.FreqUnits = 'Hz';
% opt.MagUnits  = 'abs';
% opt.MagScale  = 'log';
% opt.PhaseWrapping = 'on';
% opt.Grid = 'on';


% define quad and build path to *.bbl.csv file
flight_folder = '20231003';

quad = 'apex5';
path = ['/03_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_02.bbl.csv'];

% quad = 'apex5hd';
% path = ['/04_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];

% quad = 'flipmini';
% path = ['/05_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];

% quad = 'aosmini';
% path = ['/01_', quad, '/01_blackbox_logs/', flight_folder, '/', flight_folder, '_', quad, '_00.bbl.csv'];

file_path = ['../../../../00_quads', path];


% extract header information
[para, Nheader, ind] = extract_header_information(file_path);


% expand index
ind.axisSumPI = ind.axisError(end) + (1:3);
ind.gyro   = ind.debug(1:3);
ind.sinarg = ind.debug(4);


% read the data
tic
data = readmatrix(file_path, 'NumHeaderLines', Nheader);
Nsig = size(data, 2);
toc


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
xlim([0, max(time)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% unscale highResolutionGain
if para.blackbox_high_resolution
    blackbox_high_resolution_scale = 10.0;
    ind_bb_high_res = [ind.gyroADC,ind.gyroUnfilt,ind.rcCommand,ind.setpoint(1:3)];
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
actual_tiledlayout = tiledlayout(3,1); actual_tiledlayout.TileSpacing = 'compact';
ax(1) = nexttile;
plot(ax(1), time, data(:, [ind.setpoint(1), ind.gyro(1), ind.gyroADC(1)])), grid on, ylabel('Roll (deg/sec)')
title('Gyro Signals')
ax(2) = nexttile;
plot(ax(2), time, data(:, [ind.setpoint(2), ind.gyro(2), ind.gyroADC(2)])), grid on, ylabel('Pitch (deg/sec)')
ax(3) = nexttile;
plot(ax(3), time, data(:, [ind.setpoint(3), ind.gyro(3), ind.gyroADC(3)])), grid on, ylabel('Yaw (deg/sec)'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax, xlim([0, max(time)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% select data for spectras
data_for_spectras = data(:, [ind.gyro, ...
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
actual_tiledlayout = tiledlayout(2,1); actual_tiledlayout.TileSpacing = 'compact';
ax(1) = nexttile;
ind_show = 1:6;
plot(ax(1), freq, spectras(:, ind_show)), grid on, set(gca, 'YScale', 'log'), ylabel('Gyro (deg/sec)')
title('Spectras')
ax(2) = nexttile;
ind_show = 7:9;
plot(ax(2), freq, spectras(:, ind_show)), grid on, set(gca, 'YScale', 'log'), ylabel('AxisSum'), xlabel('Frequency (Hz)')
% ax(3) = nexttile;
% ind_show = 10:12;
% plot(ax(3), freq, spectras(:, ind_show)), grid on, set(gca, 'YScale', 'log'), ylabel('Setpoint (deg/sec)'), xlabel('Frequency (Hz)')
linkaxes(ax), clear ax, axis([0 1/2/Ts_log 1e-3 1e1])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

if (do_show_dev_figures)
    figure(22) %#ok
    actual_tiledlayout = tiledlayout(2,1); actual_tiledlayout.TileSpacing = 'compact';
    ax(1) = nexttile;
    plot(ax(1), time, data(:,ind.setpoint(1:3))), grid on, ylabel('Setpoint (deg/sec)')
    title('Chirp Excitation')
    ax(2) = nexttile;
    plot(ax(2), time, data(:,ind.sinarg)), grid on, ylabel('Sinarg (rad)'), xlabel('Time (sec)')
    linkaxes(ax, 'x'), clear ax, xlim([0, max(time)])
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
end

figure(3)
actual_tiledlayout = tiledlayout(4,1); actual_tiledlayout.TileSpacing = 'compact';
ax(1) = nexttile;
plot(ax(1), time, data(:,ind.gyro)), grid on, ylabel('Gyro (deg/sec)')
ax(2) = nexttile;
plot(ax(2), time, data(:,ind.axisSum)), grid on, ylabel('AxisSum')
ax(3) = nexttile;
plot(ax(3), time, data(:,ind.motor)), grid on, ylabel('Motora')
ax(4) = nexttile;
plot(ax(4), time, data(:,ind.setpoint(4))), grid on, ylabel('Throttle'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax
xlim([0, max(time)])
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%% frequency response estimation and calculation

Nest     = round(1.0 / Ts_log);
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


% controller frf estimates
Cpi = Gvw / (1 - T);
Cd  = Guw * Gvw / T * (1 / Guw - 1 / Gvw);

ind_freq = P.Frequency <= 1/2/Ts_log;
pos_bode = [0.1514, 0.5838-0.2, 0.7536, 0.3472+0.2]; % this is a bit hacky
pos_cohe = [0.1514, 0.1100, 0.7536, 0.1917];

if (do_show_dev_figures)
    figure(33) %#ok
    ax(1) = subplot('Position', pos_bode); opt.MagScale = 'log';
    bode(ax(1), T, 'b', 2*pi*P.Frequency(ind_freq), opt), title('Tracking T')
    ax(2) = subplot('Position', pos_cohe); opt.MagScale = 'linear';
    bodemag(ax(2), C_T, 'b', 2*pi*P.Frequency(ind_freq), opt), title(''), ylabel('Coherence')
    linkaxes(ax, 'x'), clear ax
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
    
    figure(333)
    ax(1) = subplot('Position', pos_bode); opt.MagScale = 'log';
    bode(ax(1), Guw, 'g', 2*pi*P.Frequency(ind_freq), opt), title('Guw')
    ax(2) = subplot('Position', pos_cohe); opt.MagScale = 'linear';
    bodemag(ax(2), C_Guw, 'g', 2*pi*P.Frequency(ind_freq), opt), grid on, title(''), ylabel('Coherence')
    linkaxes(ax, 'x'), clear ax
    set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)
    
    figure(3333)
    ax(1) = subplot('Position', pos_bode); opt.MagScale = 'log';
    bode(ax(1), Gvw, 'r', 2*pi*P.Frequency(ind_freq), opt), title('Gvw')
    ax(2) = subplot('Position', pos_cohe); opt.MagScale = 'linear';
    bodemag(ax(2), C_Gvw, 'r', 2*pi*P.Frequency(ind_freq), opt), title(''), ylabel('Coherence')
    linkaxes(ax, 'x'), clear ax
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
opt.YLim = {[1e-4 1e2], [-180 180]};
ax(1) = subplot('Position', pos_bode); opt.MagScale = 'log';
bode(ax(1), P/Gf_ana, 'k', 2*pi*P.Frequency(ind_freq), opt), title('Plant P'), hold on
hold off, grid on
ax(2) = subplot('Position', pos_cohe); opt.YLimMode = {'auto'}; opt.MagScale = 'linear';
bodemag(ax(2), C_T*C_Guw, 'k', 2*pi*P.Frequency(ind_freq), opt), title(''), ylabel('Coherence')
linkaxes(ax, 'x'), clear ax
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

% compare analytical to estimated controllers
figure(expand_multiple_figure_nr(8, multp_fig_nr))
opt.MagScale = 'log';
opt.YLim = {[1e-2 1e2], [-180 180]};
bode(Cpi, Cd, Cpi_ana, Cd_ana, 2*pi*P.Frequency(ind_freq), opt), title('Cpi, Cd')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


%% new controller and filter parameters

pid_axis = {'rollPID', 'pitchPID', 'yawPID'};

% PID parameters
fprintf('   used PID parameters are:\n');
fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], para.(pid_axis{ind_ax})(1:3));

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
        para_new.gyro_lowpass2_hz    = 800;     % frequency of gyro lpf 2
        para_new.gyro_soft2_type     = 0;       % type of gyro lpf 2
        para_new.gyro_notch_hz       = [0, 0]; % frequency of gyro notch 1 and 2
        para_new.gyro_notch_cutoff   = get_fcut_from_D_and_fcenter([0.00, 0.00], para_new.gyro_notch_hz); % damping of gyro notch 1 and 2
        para_new.dterm_lpf_hz        = 0;       % frequency of dterm lpf 1
        para_new.dterm_filter_type   = 0;       % type of dterm lpf 1
        para_new.dterm_lpf_dyn_hz    = [0, 0];  % dyn dterm lpf overwrites dterm_lpf_hz
        para_new.dterm_lpf2_hz       = 120;     % frequency of dterm lpf 2
        para_new.dterm_filter2_type  = 2;       % type of dterm lpf 2
        para_new.dterm_notch_hz      = 235;     % frequency of dterm notch
        para_new.dterm_notch_cutoff  = get_fcut_from_D_and_fcenter(0.15, para_new.dterm_notch_hz); % damping of dterm notch
        para_new.yaw_lpf_hz          = 200;     % frequency of yaw lpf (pt1)
        para_new.gyro_llc_freq_hz    = 100;     % frequency of gyro llc
        para_new.gyro_llc_phase      = 0;       % phase of gyro llc
        para_new.dterm_llc_freq_hz   = 100;     % frequency of dterm llc
        para_new.dterm_llc_phase     = 0;       % phase of dterm llc
        para_new.pterm_llc_freq_hz   = 100;     % frequency of pterm llc
        para_new.pterm_llc_phase     = 0;       % phase of pterm llc
        switch ind_ax
            case 1 % roll: [53, 69, 41, 41, 0]
                P_new       = 53;
                I_ratio_new = 1.15*69/69;
                D_new       = 41;
            case 2 % pitch: [65, 88, 48, 48, 0]
                P_new       = 65;
                I_ratio_new = 1.1*88/88;
                D_new       = 49;
            case 3 % yaw: [45, 85, 4, 4, 0]
                P_new       = 45;
                I_ratio_new = 1.05*85/85;
                D_new       = 4;
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

% PID parameters
fprintf('   new used PID parameters are:\n');
para_new.(pid_axis{ind_ax}) = round( PID_new ./ pid_scale);
para_new.(pid_axis{ind_ax}) = [para_new.(pid_axis{ind_ax})(1:3), ...
                               para_new.(pid_axis{ind_ax})(3), ...
                               para_new.(pid_axis{ind_ax})(4)];
fprintf(['      ', pid_axis{ind_ax}, ':  %d, %d, %d\n'], para_new.(pid_axis{ind_ax})(1:3));

[Cpi_ana_new, Cd_ana_new, Gf_ana_new, PID_new, para_used_new] = ...
    calculate_transfer_functions(para_new, ind_ax, throttle_avg, Ts_cntr);

% inform user about new parameters
para_used_fieldnames_new = fieldnames(para_used_new);
Npara_used_new = size(para_used_fieldnames_new, 1);
fprintf('   new used parameters are:\n');
for i = 1:Npara_used_new
    fprintf(['      ', para_used_fieldnames_new{i},': %d\n'], eval(['round(', 'para_used_new.', para_used_fieldnames_new{i}, ');']));
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
subplot(221)
opt.YLim = {[1e-3 1e1], [-180 180]};
bodemag(CL_ana.T , CL_ana_new.T , T, 2*pi*T.Frequency(ind_freq), opt), title('Tracking T' )
subplot(222)
bodemag(CL_ana.S , CL_ana_new.S , 2*pi*T.Frequency(ind_freq), opt), title('Sensitivity S' )
subplot(223)
opt.YLim = {[1e-2 1e2], [-180 180]};
bodemag(CL_ana.SC, CL_ana_new.SC, 2*pi*T.Frequency(ind_freq), opt), title('Controller Effort SC')
subplot(224)
opt.YLim = {[1e-4 1e0], [-180 180]};
bodemag(CL_ana.SP, CL_ana_new.SP, 2*pi*T.Frequency(ind_freq), opt), title('Compliance SP')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% step responses
f_max = 100; % needs to be set according to the coherence, if inf then it has no effect
Tmean = 0.1 * [-1, 1] + (Nest * Ts_log) / 2;
step_time = (0:Nest-1).'*Ts_log;

step_resp = [calculate_step_response_from_frd(CL_ana.T    , f_max), ...
             calculate_step_response_from_frd(CL_ana_new.T, f_max), ...
             calculate_step_response_from_frd(T           , f_max)];
step_resp_mean = mean(step_resp(step_time > Tmean(1) & step_time < Tmean(2),:));
step_resp = step_resp ./ step_resp_mean;

figure(expand_multiple_figure_nr(6, multp_fig_nr))
actual_tiledlayout = tiledlayout(2,1); actual_tiledlayout.TileSpacing = 'compact';
ax(1) = nexttile;
plot(ax(1), step_time, step_resp(:,1)), grid on, hold on
plot(ax(1), step_time, step_resp(:,2))
plot(ax(1), step_time, step_resp(:,3), 'r'), hold off
xlim([0 0.5]), ylim([0 1.3])
ylabel('Gyro (deg/sec)'), title('Tracking T')

step_resp = [calculate_step_response_from_frd(CL_ana.SP    , f_max), ...
             calculate_step_response_from_frd(CL_ana_new.SP, f_max)];
step_resp_mean = mean(step_resp(step_time > Tmean(1) & step_time < Tmean(2),:));
step_resp = step_resp - step_resp_mean;

ax(2) = nexttile;
plot(ax(2), step_time, step_resp(:,1), 'b'), grid on, hold on
plot(ax(2), step_time, step_resp(:,2), 'color', [0 0.5 0]), hold off
linkaxes(ax, 'x'), clear ax, xlim([0 0.5]), ylim([-0.2 1.1])
xlabel('Time (sec)'), ylabel('Gyro (deg/sec)'), title('Compliance SP')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


figure(expand_multiple_figure_nr(7, multp_fig_nr))
opt.YLim = {[1e-1 1e2], [-180 180]};
bode(CL_ana.C, CL_ana_new.C, 2*pi*T.Frequency(ind_freq), opt)
title('Controller C')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% group and phase delay tracking
phase       = unwrap(angle(squeeze(CL_ana.T.ResponseData)));
% group_delay = -gradient(phase, 2*pi*CL_ana.T.Frequency);
phase_delay = -phase ./ (2*pi*CL_ana.T.Frequency);
phase_new       = unwrap(angle(squeeze(CL_ana_new.T.ResponseData)));
% group_delay_new = -gradient(phase_new, 2*pi*CL_ana_new.T.Frequency);
phase_delay_new = -phase_new ./ (2*pi*CL_ana_new.T.Frequency);

figure(expand_multiple_figure_nr(8, multp_fig_nr))
% subplot(121)
% semilogx(T.Frequency(ind_freq), [group_delay(ind_freq), group_delay_new(ind_freq)] * 1e3), grid on
% xlim([0 100]), ylim([0 30])
% xlabel('Frequency (Hz)'), ylabel('Group Delay (ms)'), title('Group Delay T')
% subplot(122)
semilogx(T.Frequency(ind_freq), [phase_delay(ind_freq), phase_delay_new(ind_freq)] * 1e3), grid on
xlim([0 100]), ylim([0 30])
xlabel('Frequency (Hz)'), ylabel('Phase Delay (ms)'), title('Phase Delay Tracking T')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)


% open-loop
figure(expand_multiple_figure_nr(9, multp_fig_nr))
opt.YLim = {[1e-2 1e2], [-180 180]};
bode(CL_ana.L, CL_ana_new.L, 2*pi*T.Frequency(ind_freq), opt), title('Open-Loop')
set(findall(gcf, 'type', 'line'), 'linewidth', linewidth)

