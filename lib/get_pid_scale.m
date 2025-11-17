function pid_scale = get_pid_scale(ind_ax)

    PTERM_SCALE = 0.032029;
    ITERM_SCALE = 0.244381;
    DTERM_SCALE = 0.000529;
    % FEEDFORWARD_SCALE = 0.013754;

    pid_scale = [PTERM_SCALE, ITERM_SCALE, DTERM_SCALE];

    if ind_ax == 3
        pid_scale(2) = pid_scale(2) * 2.5;
    end

end
