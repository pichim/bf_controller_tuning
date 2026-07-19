function opt = bode_plot_options(mag_unit, mag_scale, phase_unit, freq_unit)
% bode_plot_options  Create a set of bode plot options.
%
% Inputs:
%   mag_unit   - 'dB' or 'abs'
%   mag_scale  - 'linear' or 'log'
%   phase_unit - 'deg' or 'rad'
%
% Output:
%   opt        - bodeoptions object

    % Start with default Bode plot preferences
    opt = bodeoptions('cstprefs');

    %% --- Validate magnitude unit ---
    if strcmp(mag_unit, 'dB')
        opt.MagUnits = 'dB';
    elseif strcmp(mag_unit, 'abs')
        opt.MagUnits = 'abs';
    else
        error("Invalid mag_unit. Use 'dB' or 'abs'.");
    end

    %% --- Validate magnitude scale (y-axis scale) ---
    if strcmp(mag_scale, 'log')
        opt.MagScale = 'log';
    elseif strcmp(mag_scale, 'linear')
        opt.MagScale = 'linear';
    else
        error("Invalid mag_scale. Use 'linear' or 'log'.");
    end

    %% --- Validate phase unit ---
    if strcmp(phase_unit, 'deg')
        opt.PhaseUnits = 'deg';
    elseif strcmp(phase_unit, 'rad')
        opt.PhaseUnits = 'rad';
    else
        error("Invalid phase_unit. Use 'deg' or 'rad'.");
    end

    %% --- Validate frequency unit ---
    if strcmp(freq_unit, 'Hz')
        opt.FreqUnits = 'Hz';
    elseif strcmp(freq_unit, 'rad/s')
        opt.FreqUnits = 'rad/s';
    else
        error("Invalid freq_unit. Use 'Hz' or 'rad/s'.");
    end

    %% --- Useful defaults ---
    opt.PhaseWrapping = 'on';   % Wrap phase into [-180, 180] or [-pi, pi]
    opt.Grid = 'on';            % Enable grid

end
