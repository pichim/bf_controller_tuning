"""
==========================================================================
GYRO CTRL TUNING - Betaflight Controller Analysis (Gyro tuning class)
==========================================================================

Purpose:
    Calculations for gyro tuning

Authors:
    Yuri Bianchi
    Janick Dort
    Dario Jurietti

Supervisors:
    Michael Peter
    Prof. Dr. Ruprecht Altenburger

Date:
    05.06.2026
==========================================================================
"""

import numpy as np
import control as ct
import copy
import time

from scipy.signal.windows import hann
from typing import Optional, List

from .pidtuninglib import (
    header_info,
    ClosedLoop,
    apply_rotfiltfilt,
    estimate_frequency_response,
    calculate_transfer_functions,
    calculate_closed_loop,
    downsample_frd,
    calculate_step_response_from_frd,
    get_pid_scale,
)


def get_ind_eval(
    sinarg: np.ndarray,
    gyro_data: np.ndarray,
    threshold: float = 500.0
) -> np.ndarray:
    """
    Select evaluation intervals based on excitation activity
    and signal variance.
    """

    ndata = len(sinarg)

    # =============================================================
    # Detect Excitation Regions
    # =============================================================

    ind_eval_candidate = sinarg > 0

    signal = np.zeros(ndata)
    signal[ind_eval_candidate] = 1

    dsignal = np.diff(signal, prepend=0)

    ind_eval_start = np.where(dsignal > 0.9)[0]
    ind_eval_end = np.where(dsignal < -0.9)[0] - 1

    # =============================================================
    # Validate Regions
    # =============================================================

    ind_eval = np.zeros(ndata, dtype=bool)

    for start, end in zip(ind_eval_start, ind_eval_end):

        ind_verify = slice(start, end + 1)

        # Only use intervals with sufficient gyro activity
        if np.var(gyro_data[ind_verify]) > threshold:
            ind_eval[ind_verify] = True

    return ind_eval


class GyroCtrlTuning:
    """
    Gyro controller tuning and analysis.

    Estimates and calculates frequency responses for system
    identification and controller analysis.

    Key components analyzed:
        T   : Complementary sensitivity
        P   : Plant
        Cpi : PI controller
        Cd  : D controller

    Signal definitions:
        w : reference input (setpoint)
        y : system output (gyro measurements)
        u : control input (total PID output)
        v : PI controller output
    """

    def __init__(
        self,
        data: np.ndarray,
        ind,
        Ts_log: float,
        para: header_info,
        Ts_cntr: float
    ):

        # =============================================================
        # Flight Data
        # =============================================================

        self.data = data
        self.ind = ind

        self.Ts_log = Ts_log
        self.Ts_cntr = Ts_cntr

        self.para = para

        # =============================================================
        # General Plotting
        # =============================================================

        self.linewidth = 1.2

        # =============================================================
        # Transfer Function Data
        # =============================================================

        self.T: List[ct.FRD] = []
        self.Guw: List[ct.FRD] = []
        self.Gvw: List[ct.FRD] = []

        self.P_gef: List[ct.FRD] = []
        self.P: List[ct.FRD] = []

        self.Cpi: List[ct.FRD] = []
        self.Cd: List[ct.FRD] = []

        # =============================================================
        # Analytical Transfer Functions
        # =============================================================

        self.Gf_ana: List[ct.FRD] = []
        self.Cpi_ana: List[ct.FRD] = []
        self.Cd_ana: List[ct.FRD] = []

        # =============================================================
        # PID and Parameter Data
        # =============================================================

        self.PID: List[np.ndarray] = []
        self.para_used: List[header_info] = []

        # Frequency vector for bode plots
        self.omega_bode: Optional[np.ndarray] = None

        # =============================================================
        # Coherence
        # =============================================================

        self.Coh_T: List[ct.FRD] = []
        self.Coh_C: List[ct.FRD] = []
        self.Coh_P: List[ct.FRD] = []

        # =============================================================
        # General Analysis Data
        # =============================================================

        self.throttle_avg: Optional[float] = None

        self.Nest: Optional[int] = None

        self.ind_ax: Optional[int] = None

        # =============================================================
        # New Controller Data
        # =============================================================

        self.Cpi_ana_new: Optional[ct.FRD] = None
        self.Gf_ana_new: Optional[ct.FRD] = None
        self.Cd_ana_new: Optional[ct.FRD] = None

        # =============================================================
        # Closed Loop Data
        # =============================================================

        self.CL_ana: Optional[ClosedLoop] = None
        self.CL_ana_new: Optional[ClosedLoop] = None

        # =============================================================
        # Step Response Data
        # =============================================================

        self.step_time: Optional[np.ndarray] = None

        self.step_resp_tra: Optional[np.ndarray] = None
        self.step_resp_com: Optional[np.ndarray] = None

    def calculate_transfer_func(
        self,
        Nestfatra: float,
        koverlaptra: float
    ) -> "GyroCtrlTuning":

        # =============================================================
        # Analysis Window Parameters
        # =============================================================
        # Nest:
        #     Window length in samples
        #
        # Noverlap:
        #     Overlap between windows
        # =============================================================

        self.Nest = int(np.round(Nestfatra / self.Ts_log))

        Noverlap = int(np.floor(koverlaptra * self.Nest))

        # Hanning window for spectral analysis
        window = hann(self.Nest, sym=False)

        # =============================================================
        # Excitation Filter
        # =============================================================
        # Design linear filter for zero-phase excitation filtering
        # (apply_rotfiltfilt)
        # =============================================================

        Dlp = np.sqrt(3) / 2       # Damping ratio
        wlp = 2 * np.pi * 10       # Cutoff frequency [rad/s]

        Glp_cont = ct.tf(
            [wlp**2],
            [1, 2 * Dlp * wlp, wlp**2]
        )

        # Discrete filter using Tustin transform
        Glp = ct.c2d(
            Glp_cont,
            self.Ts_log,
            method='tustin'
        )

        # =============================================================
        # Preallocate Storage
        # =============================================================

        n_axes = 3

        self.T = [None] * n_axes
        self.Guw = [None] * n_axes
        self.Gvw = [None] * n_axes

        self.P_gef = [None] * n_axes
        self.P = [None] * n_axes

        self.Cpi = [None] * n_axes
        self.Cd = [None] * n_axes

        self.Coh_T = [None] * n_axes
        self.Coh_C = [None] * n_axes
        self.Coh_P = [None] * n_axes

        self.Gf_ana = [None] * n_axes
        self.Cpi_ana = [None] * n_axes
        self.Cd_ana = [None] * n_axes

        self.PID = [None] * n_axes
        self.para_used = [None] * n_axes

        # =============================================================
        # Process All Axes
        # =============================================================
        # 0 = Roll
        # 1 = Pitch
        # 2 = Yaw
        # =============================================================

        for ind_axis in range(n_axes):

            # =========================================================
            # Select Valid Evaluation Regions
            # =========================================================

            ind_eval = get_ind_eval(
                self.data[:, self.ind.sinarg],
                self.data[:, self.ind.gyroADC[ind_axis]]
            )

            sinarg_ax = self.data[:, self.ind.sinarg].copy()
            sinarg_ax[~ind_eval] = 0

            # =========================================================
            # Average Throttle
            # =========================================================

            self.throttle_avg = np.median(
                self.data[ind_eval, self.ind.setpoint[3]]
            ) / 1.0e3

            # =========================================================
            # Input Signal
            # =========================================================
            # w:
            #     Filtered setpoint for selected axis
            # =========================================================

            w = self.data[:, self.ind.setpoint[ind_axis]]

            inp = apply_rotfiltfilt(
                Glp,
                sinarg_ax,
                w[:, np.newaxis]
            )

            # =========================================================
            # Output Signal
            # =========================================================
            # y:
            #     Filtered gyro signal
            # =========================================================

            y = self.data[:, self.ind.gyroADC[ind_axis]]

            out_y = apply_rotfiltfilt(
                Glp,
                sinarg_ax,
                y[:, np.newaxis]
            )

            # =========================================================
            # Complementary Sensitivity
            # =============================================================
            # T:
            #     Response from reference input to output
            #
            # Gyw:
            #     w -> y
            # =============================================================

            T_ax, C_T_ax, freq_hz, _ = estimate_frequency_response(
                inp[ind_eval].ravel(),
                out_y[ind_eval].ravel(),
                window,
                Noverlap,
                self.Nest,
                self.Ts_log
            )

            # =========================================================
            # Control Sensitivity
            # =============================================================
            # Guw:
            #     Response from reference input to controller output
            #
            # Guw:
            #     w -> u
            # =============================================================

            u = self.data[:, self.ind.axisSum[ind_axis]]

            out_u = apply_rotfiltfilt(
                Glp,
                sinarg_ax,
                u[:, np.newaxis]
            )

            Guw_ax, C_Guw_ax, _, _ = estimate_frequency_response(
                inp[ind_eval].ravel(),
                out_u[ind_eval].ravel(),
                window,
                Noverlap,
                self.Nest,
                self.Ts_log
            )

            # =========================================================
            # PI Controller Response
            # =============================================================
            # Gvw:
            #     Response from reference input to PI controller output
            #
            # Gvw:
            #     w -> v
            # =============================================================

            v = self.data[:, self.ind.axisSumPI[ind_axis]]

            out_v = apply_rotfiltfilt(
                Glp,
                sinarg_ax,
                v[:, np.newaxis]
            )

            Gvw_ax, _, _, _ = estimate_frequency_response(
                inp[ind_eval].ravel(),
                out_v[ind_eval].ravel(),
                window,
                Noverlap,
                self.Nest,
                self.Ts_log
            )

            # =========================================================
            # Plant Estimation
            # =============================================================

            P_gef_ax = T_ax / Guw_ax

            # Combined coherence
            Coh_Plant = C_T_ax * C_Guw_ax

            # =========================================================
            # Controller Estimation
            # =============================================================
            # Split controller into PI and D components
            # =============================================================

            Cpi_ax = Gvw_ax / (1 - T_ax)

            Cd_ax = (
                Guw_ax
                * Gvw_ax
                / T_ax
                * (1 / Guw_ax - 1 / Gvw_ax)
            )

            # =========================================================
            # Frequency Vector
            # =============================================================

            omega_bode_ax = T_ax.omega

            # =========================================================
            # Store Results
            # =============================================================

            self.T[ind_axis] = T_ax
            self.Guw[ind_axis] = Guw_ax
            self.Gvw[ind_axis] = Gvw_ax

            self.P_gef[ind_axis] = P_gef_ax

            self.Cpi[ind_axis] = Cpi_ax
            self.Cd[ind_axis] = Cd_ax

            self.omega_bode = omega_bode_ax

            self.Coh_T[ind_axis] = C_T_ax
            self.Coh_C[ind_axis] = C_Guw_ax
            self.Coh_P[ind_axis] = Coh_Plant

            # =========================================================
            # Analytical Transfer Functions
            # =============================================================

            (
                Cpi_ana_ax,
                Cd_ana_ax,
                Gf_ana_ax,
                PID_ax,
                para_used_ax
            ) = calculate_transfer_functions(
                self.para,
                ind_axis,
                self.throttle_avg,
                self.Ts_cntr
            )

            # =========================================================
            # Downsample if Necessary
            # =============================================================
            # Downsample analytical transfer functions to logging rate
            # =============================================================

            if Gf_ana_ax.dt < self.Ts_log:

                freq_target = T_ax.omega / (2 * np.pi)

                Gf_ana_ax = downsample_frd(
                    Gf_ana_ax,
                    self.Ts_log,
                    freq_target
                )

                Cpi_ana_ax = downsample_frd(
                    Cpi_ana_ax,
                    self.Ts_log,
                    freq_target
                )

                Cd_ana_ax = downsample_frd(
                    Cd_ana_ax,
                    self.Ts_log,
                    freq_target
                )

            # =========================================================
            # Store Analytical Results
            # =============================================================

            self.Gf_ana[ind_axis] = Gf_ana_ax
            self.Cpi_ana[ind_axis] = Cpi_ana_ax
            self.Cd_ana[ind_axis] = Cd_ana_ax

            self.PID[ind_axis] = PID_ax
            self.para_used[ind_axis] = para_used_ax

            # Estimated real plant
            self.P[ind_axis] = P_gef_ax / Gf_ana_ax

        return self

    def calculate_new_controller(
        self,
        ind_ax: int,
        P_new: float,
        I_new: float,
        D_new: float,
        default_parameters: bool,
        para_new: header_info
    ) -> "GyroCtrlTuning":

        start_time = time.time()

        # Axis names used in Betaflight
        pid_axis = ['rollPID', 'pitchPID', 'yawPID']

        # =============================================================
        # Default Parameters
        # =============================================================

        if default_parameters:

            para_new = self.para.make_copy()

            pid_vec = self.para.get_list(pid_axis[ind_ax])

            # Use current PID values
            P_new = pid_vec[0]
            I_new = pid_vec[1]
            D_new = pid_vec[2]

        # =============================================================
        # PID Scaling
        # =============================================================
        # Roll, pitch and yaw use different internal scaling
        # =============================================================

        pid_scale = get_pid_scale(ind_ax) + [1]

        PID_new = np.zeros(4)

        # P Gain
        PID_new[0] = P_new * pid_scale[0]

        # Current I frequency
        fI = (
            self.PID[ind_ax][1]
            / (2 * np.pi * self.PID[ind_ax][0])
        )

        # I Gain
        PID_new[1] = I_new * pid_scale[1]

        # New I frequency
        fI_new = (
            PID_new[1]
            / (2 * np.pi * PID_new[0])
        )

        # D Gain
        PID_new[2] = D_new * pid_scale[2]

        # No feedforward
        PID_new[3] = 0

        # =============================================================
        # Store New Parameters
        # =============================================================

        para_new_pid = np.round(
            PID_new / pid_scale
        ).astype(int)

        para_new_list = [
            para_new_pid[0],
            para_new_pid[1],
            para_new_pid[2],
            para_new_pid[2],
            para_new_pid[3]
        ]

        para_new.data[pid_axis[ind_ax]] = ",".join(
            map(str, para_new_list)
        )

        # =============================================================
        # Calculate New Transfer Functions
        # =============================================================

        (
            self.Cpi_ana_new,
            self.Cd_ana_new,
            self.Gf_ana_new,
            PID_new,
            para_used_new
        ) = calculate_transfer_functions(
            para_new,
            ind_ax,
            self.throttle_avg,
            self.Ts_cntr
        )

        # =============================================================
        # Downsample if Necessary
        # =============================================================

        if self.Gf_ana_new.dt < self.Ts_log:

            freq_target = (
                self.P_gef[ind_ax].omega
                / (2 * np.pi)
            )

            self.Gf_ana_new = downsample_frd(
                self.Gf_ana_new,
                self.Ts_log,
                freq_target
            )

            self.Cpi_ana_new = downsample_frd(
                self.Cpi_ana_new,
                self.Ts_log,
                freq_target
            )

            self.Cd_ana_new = downsample_frd(
                self.Cd_ana_new,
                self.Ts_log,
                freq_target
            )

        self.ind_ax = ind_ax

        print(
            f"Calculation time: "
            f"{time.time() - start_time:.3f}s"
        )

        return self

    def get_tuning_data(
        self,
        do_compensate_iterm: bool
    ) -> "GyroCtrlTuning":

        start_time = time.time()

        # =============================================================
        # Closed Loop Responses
        # =============================================================
        # Calculate closed loop responses for:
        #   - current parameters
        #   - new parameters
        # =============================================================

        self.CL_ana = calculate_closed_loop(
            self.Cpi_ana[self.ind_ax],
            ct.tf([1], [1], self.Ts_log),
            self.P[self.ind_ax],
            self.Gf_ana[self.ind_ax],
            self.Cd_ana[self.ind_ax]
        )

        self.CL_ana_new = calculate_closed_loop(
            self.Cpi_ana_new,
            ct.tf([1], [1], self.Ts_log),
            self.P[self.ind_ax],
            self.Gf_ana_new,
            self.Cd_ana_new
        )

        # =============================================================
        # I-Term Compensation
        # =============================================================
        # Compensate only PI controller part
        # =============================================================

        if do_compensate_iterm:

            Cpi_com = (
                self.Cpi[self.ind_ax]
                / self.Cpi_ana[self.ind_ax]
            )

            CL_ana_ = calculate_closed_loop(
                self.Cpi_ana[self.ind_ax] * Cpi_com,
                ct.tf([1], [1], self.Ts_log),
                self.P[self.ind_ax],
                self.Gf_ana[self.ind_ax],
                self.Cd_ana[self.ind_ax]
            )

            CL_ana_new_ = calculate_closed_loop(
                self.Cpi_ana_new * Cpi_com,
                ct.tf([1], [1], self.Ts_log),
                self.P[self.ind_ax],
                self.Gf_ana_new,
                self.Cd_ana_new
            )

            # Overwrite complementary sensitivity only
            self.CL_ana.T = CL_ana_.T
            self.CL_ana_new.T = CL_ana_new_.T

        # =============================================================
        # Step Response Analysis
        # =============================================================
        # Compare:
        #   - current tuning
        #   - new tuning
        #   - measured response
        # =============================================================

        # Frequency limit based on notch settings
        f_max = min(
            self.para.get_int('dyn_notch_min_hz'),
            self.para.get_int('gyro_rpm_notch_min')
        )

        # Time window for normalization
        T_mean = (
            0.1 * np.array([-1, 1])
            + (self.Nest * self.Ts_log) / 2
        )

        # Time vector
        self.step_time = (
            np.arange(self.Nest)
            * self.Ts_log
        )

        # =============================================================
        # Tracking Performance
        # =============================================================

        step_resp = np.column_stack([

            # Used parameters
            calculate_step_response_from_frd(
                copy.deepcopy(self.CL_ana.T),
                f_max
            ),

            # New parameters
            calculate_step_response_from_frd(
                copy.deepcopy(self.CL_ana_new.T),
                f_max
            ),

            # Measured response
            calculate_step_response_from_frd(
                copy.deepcopy(self.T[self.ind_ax]),
                f_max
            )
        ])

        mask = (
            (self.step_time > T_mean[0])
            & (self.step_time < T_mean[1])
        )

        # Normalize responses
        step_resp_mean = np.mean(
            step_resp[mask, :],
            axis=0
        )

        step_resp = step_resp / step_resp_mean

        self.step_resp_tra = step_resp

        # =============================================================
        # Disturbance Rejection
        # =============================================================

        step_resp = np.column_stack([

            # Used parameters
            calculate_step_response_from_frd(
                copy.deepcopy(self.CL_ana.SP),
                f_max
            ),

            # New parameters
            calculate_step_response_from_frd(
                copy.deepcopy(self.CL_ana_new.SP),
                f_max
            )
        ])

        # Center responses around mean
        step_resp_mean = np.mean(
            step_resp[mask, :],
            axis=0
        )

        step_resp = step_resp - step_resp_mean

        self.step_resp_com = step_resp

        print(
            f"Tuning data calculation time: "
            f"{time.time() - start_time:.3f}s"
        )

        return self