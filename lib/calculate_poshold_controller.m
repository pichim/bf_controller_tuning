function [Cpid] = calculate_poshold_controller(P, I, D, A, fc_pt1, Ts_cntr, Ts_log, freq)

    AP_P_SCALE = 0.0012;
    AP_I_SCALE = 0.0001;
    AP_D_SCALE = 0.0015;
    AP_A_SCALE = 0.0008;

    Kp = P * AP_P_SCALE;
    Ki = I * AP_I_SCALE;
    Kd = D * AP_D_SCALE;
    Ka = A * AP_A_SCALE;

    z = tf('z', Ts_cntr);

    % PT1 filter for velocity and acceleration
    Gpt1 = get_filter('pt1', fc_pt1, Ts_cntr);

    % Discrete backward differentiator: (z - 1) / (Ts*z)
    diff_z = (z - 1) / (Ts_cntr * z);

    % Discrete forward Euler integrator: Ts*z / (z - 1)
    int_z = Ts_cntr * z / (z - 1);

    % PIDA controller
    Cpid_tf = Kp ...
            + Ki * int_z ...function [Cpid] = calculate_poshold_controller(P, I, D, A, fc_pt1, Ts_cntr, Ts_log, freq)

    AP_P_SCALE = 0.0012;
    AP_I_SCALE = 0.0001;
    AP_D_SCALE = 0.0015;
    AP_A_SCALE = 0.0008;

    Kp = P * AP_P_SCALE;
    Ki = I * AP_I_SCALE;
    Kd = D * AP_D_SCALE;
    Ka = A * AP_A_SCALE;

    z = tf('z', Ts_cntr);

    % PT1 filter for velocity and acceleration
    Gpt1 = get_filter('pt1', fc_pt1, Ts_cntr);

    % Discrete backward differentiator: (z - 1) / (Ts*z)
    diff_z = (z - 1) / (Ts_cntr * z);

    % Discrete forward Euler integrator: Ts*z / (z - 1)
    int_z = Ts_cntr * z / (z - 1);

    % PIDA controller
    Cpid_tf = Kp ...
            + Ki * int_z ...
            + Kd * diff_z * Gpt1 ...
            + Ka * diff_z^2 * Gpt1^2;

    % Convert to FRD at desired frequencies
    Cpid = frd(Cpid_tf, 2*pi*freq);

    % Optional: match returned FRD sample time to log sample time
    Cpid = downsample_frd(Cpid, Ts_log, freq);

end
