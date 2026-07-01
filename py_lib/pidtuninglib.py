"""
==========================================================================
PID Tuning Library
==========================================================================

Purpose:
    Core control theory and signal processing functions for offline PID 
    controller analysis and tuning from Betaflight flight logs.

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
import warnings
import control as ct
from numpy.typing import NDArray
from scipy.signal import convolve2d, filtfilt
from typing import Union, List, Dict
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class header_info:
    """Flight controller parameter storage with type conversion helpers."""

    data: Dict[str, str] = field(default_factory=dict)

    def __getitem__(self, key: str) -> str:
        if key not in self.data:
            raise ValueError(f"parameter: {key} not found!")
        return self.data[key]

    def __setitem__(self, key: str, value: str) -> None:
        if key not in self.data:
            raise KeyError(f"key {key} does not exist!")
        self.data[key] = value

    def create_item(self, key: str, value: str) -> None:
        if key in self.data:
            raise KeyError(f"key {key} already exists!")
        self.data[key] = value

    def mod_list(self, key: str, new_value: str, idx: int = 0) -> None:
        """Modify comma-separated list value at specified index."""
        if key not in self.data:
            raise KeyError(f"key: {key} not found!")
        lst = self[key].split(",")
        if idx > len(lst):
            raise ValueError("index out of bounds")
        lst[idx] = new_value
        self.data[key] = ",".join(str(x) for x in lst)

    def make_copy(self) -> "header_info":
        return header_info(data=self.data.copy())

    def get_float(self, key: str) -> float:
        return float(self[key])

    def get_int(self, key: str) -> int:
        return int(self[key])

    def get_list(self, key: str, dtype=int) -> list:
        return [dtype(x) for x in self[key].split(",")]

    @classmethod
    def get_header(cls, file_path: Path) -> "header_info":
        """Extract flight log header parameters (lines 11-153)."""
        data = {}
        with file_path.open("r") as f:
            for idx, line in enumerate(f):
                if idx > 10:
                    parts = line.strip().split(",", 1)
                    if len(parts) == 2:
                        key = parts[0].strip('"')
                        value = parts[1].strip('"')
                        data[key] = value
                if idx > 153:
                    break
        return cls(data=data)


@dataclass
class ClosedLoop:
    """All closed-loop transfer functions for control analysis."""

    C: ct.FRD  # Total controller
    L: ct.FRD  # Loop transfer function
    S: ct.FRD  # Sensitivity
    SCw: ct.FRD  # Outer controller sensitivity
    T: ct.FRD  # Complementary sensitivity (reference tracking)
    SP: ct.FRD  # Input disturbance to output
    SC: ct.FRD  # Noise to control signal
    Li: ct.FRD  # Inner loop transfer
    Pi: ct.FRD  # Inner closed loop from outer controller view
    Ti: ct.FRD  # Inner complementary sensitivity
    Si: ct.FRD  # Inner sensitivity
    Lo: ct.FRD  # Outer loop transfer


def apply_rotfiltfilt(
    G: ct.TransferFunction, sinarg: NDArray[np.floating], x: NDArray[np.floating]
) -> NDArray[np.floating]:
    """Apply zero-phase filter with rotating frame method to avoid edge effects."""
    Nx, nx = x.shape
    xf = np.zeros((Nx, nx))
    p = np.exp(1j * sinarg)

    num = G.num[0][0]
    den = G.den[0][0]

    for i in range(nx):
        y = x[:, i] - np.mean(x[:, i])
        # Rotate signal to DC, filter, rotate back
        yR = y * p
        yQ = y * np.conj(p)

        yR = filtfilt(num, den, yR)
        yQ = filtfilt(num, den, yQ)

        xf[:, i] = np.real((yR * np.conj(p) + yQ * p) / 2)

    return xf


def calculate_closed_loop(
    Co: ct.FRD, Ci: ct.TransferFunction, P: ct.FRD, Gf: ct.FRD, Gd: ct.FRD
) -> ClosedLoop:
    """
    Compute all cascade control loop transfer functions.
    Co: outer controller, Ci: inner PID, P: plant, Gf: gyro filters, Gd: D-term filters
    """
    C = Ci * (Gd + Co) * Gf
    L = P * C
    S = 1 / (1 + L)

    T = Co * Ci * P * S
    SP = P * S
    SC = C * S
    SCw = Co * Ci * S

    Li = Ci * P * Gf * Gd
    Si = ct.frd(1 / (1 + Li))
    Pi = Ci * P * Gf * Si
    Ti = Li * Si

    Lo = Co * Ci * P * Gf / (1 + Ci * P * Gf * Gd)

    return ClosedLoop(C, L, S, SCw, T, SP, SC, Li, Pi, Ti, Si, Lo)

def calculate_closed_loop_angle(
    C: ct.FRD,
    T_gyro: ct.FRD,
    P_gyro: ct.FRD,
    P: ct.FRD,
    C_gyro: ct.FRD
) -> ClosedLoop:

    L = P * C * T_gyro
    S = 1 / (1 + L)

    T = L * S
    SP = S * P
    SC = C * S

    return ClosedLoop(
        C,
        L,
        S,
        None,
        T,
        SP,
        SC,
        None,
        None,
        None,
        None,
        None
    )




def calculate_controllers(PID, Gf_p, Ts):
    """Build discrete-time PID controllers from gains."""
    Kp = float(PID[0])
    Ki = float(PID[1])
    Kd = float(PID[2])

    integrator = ct.tf([1, 0], [1, -1], Ts)
    Cpi_temp = Kp * Gf_p + Ki * Ts * integrator
    Cpi = ct.ss(Cpi_temp)

    differentiator = ct.tf([1, -1], [1, 0], Ts)
    Cd_temp = (Kd / Ts) * differentiator
    Cd = ct.ss(Cd_temp)

    return Cpi, Cd


def calculate_step_response_from_frd(
    G: ct.FRD, f_max_hz: float
) -> NDArray[np.floating]:
    """Convert FRD to step response via inverse FFT with bandwidth limit."""
    g = np.squeeze(G.fresp)
    freq = G.omega / (2 * np.pi)

    if np.isnan(np.abs(g[0])):
        g[0] = g[1]

    g[freq > f_max_hz] = 0

    # Mirror spectrum for real IFFT
    g_mirror = np.conj(g[-2:0:-1])
    g_full = np.concatenate((g, g_mirror))

    step_resp = np.cumsum(np.real(np.fft.ifft(g_full)))

    return step_resp


def calculate_transfer_functions(
    para: header_info, ind_ax: int, throttle_avg: float, Ts: float
):
    """
    Build all filter and controller transfer functions from flight log parameters.
    ind_ax: 0=roll, 1=pitch, 2=yaw
    """
    filter_types = ["pt1", "biquad", "pt2", "pt3"]

    Gf = ct.ss(ct.tf(1, 1, Ts))
    para_used = para.make_copy()

    # Gyro lowpass filter 1
    if para.get_int("gyro_lowpass_hz") > 0:
        para_used["gyro_lowpass_hz"] = para["gyro_lowpass_hz"]
        para_used["gyro_soft_type"] = para["gyro_soft_type"]
        Gf = (
            Gf
            * get_filter(
                filter_types[para.get_int("gyro_soft_type")],
                para.get_int("gyro_lowpass_hz"),
                Ts,
            )[0]
        )

    # Dynamic gyro lowpass (throttle-dependent cutoff)
    if para.get_list("gyro_lowpass_dyn_hz")[0] > 0:
        Gf = ct.ss(ct.tf(1, 1, Ts))
        para_used["gyro_lowpass_dyn_hz"] = para["gyro_lowpass_dyn_hz"]
        para_used["gyro_soft_type"] = para["gyro_soft_type"]
        para_used.create_item(
            "gyro_lpf_hz_avg",
            str(
                get_fcut_from_exp(
                    para.get_list("gyro_lowpass_dyn_hz")[0],
                    para.get_list("gyro_lowpass_dyn_hz")[1],
                    para.get_int("gyro_lowpass_dyn_expo"),
                    throttle_avg,
                )
            ),
        )
        para_used.create_item("gyro_lpf_throttle_avg", str(throttle_avg))
        Gf = (
            Gf
            * get_filter(
                filter_types[para.get_int("gyro_soft_type")],
                para_used.get_float("gyro_lpf_hz_avg"),
                Ts,
            )[0]
        )

    # Gyro lowpass filter 2
    if para.get_int("gyro_lowpass2_hz") > 0:
        para_used["gyro_lowpass2_hz"] = para["gyro_lowpass2_hz"]
        para_used["gyro_soft2_type"] = para["gyro_soft2_type"]
        Gf = ct.series(
            Gf,
            get_filter(
                filter_types[para.get_int("gyro_soft2_type")],
                para.get_int("gyro_lowpass2_hz"),
                Ts,
            )[0],
        )

    # Gyro notch filters 1 & 2
    if para.get_list("gyro_notch_hz")[0] > 0:
        para_used.mod_list("gyro_notch_hz", str(para.get_list("gyro_notch_hz")[0]), 0)
        para_used.mod_list(
            "gyro_notch_cutoff", str(para.get_list("gyro_notch_cutoff")[0]), 0
        )
        Gf = (
            Gf
            * get_filter(
                "notch",
                [
                    para.get_list("gyro_notch_cutoff")[0],
                    para.get_list("gyro_notch_hz")[0],
                ],
                Ts,
            )[0]
        )

    if para.get_list("gyro_notch_hz")[1] > 0:
        para_used.mod_list("gyro_notch_hz", str(para.get_list("gyro_notch_hz")[1]), 1)
        para_used.mod_list(
            "gyro_notch_cutoff", str(para.get_list("gyro_notch_cutoff")[1]), 1
        )
        Gf = (
            Gf
            * get_filter(
                "notch",
                [
                    para.get_list("gyro_notch_cutoff")[1],
                    para.get_list("gyro_notch_hz")[1],
                ],
                Ts,
            )[0]
        )

    # Gyro linear phase compensation
    if "gyro_llc_freq_hz" in para.data:
        if para.get_int("gyro_llc_phase") != 0:
            para_used["gyro_llc_freq_hz"] = para["gyro_llc_freq_hz"]
            para_used["gyro_llc_phase"] = para["gyro_llc_phase"]
            Gf = (
                Gf
                * get_filter(
                    "phaseComp",
                    [para.get_int("gyro_llc_freq_hz"), para.get_int("gyro_llc_phase")],
                    Ts,
                )[0]
            )

    # D-term filters (applied to derivative of gyro signal)
    Gd = ct.ss(ct.tf(1, 1, Ts))

    if para.get_int("dterm_lpf_hz") > 0:
        para_used["dterm_lpf_hz"] = para["dterm_lpf_hz"]
        para_used["dterm_filter_type"] = para["dterm_filter_type"]
        Gd = (
            Gd
            * get_filter(
                filter_types[para.get_int("dterm_filter_type")],
                para.get_int("dterm_lpf_hz"),
                Ts,
            )[0]
        )

    # Dynamic D-term lowpass
    if para.get_list("dterm_lpf_dyn_hz")[0] > 0:
        Gd = ct.ss(ct.tf(1, 1, Ts))
        para_used["dterm_lpf_dyn_hz"] = para["dterm_lpf_dyn_hz"]
        para_used["dterm_filter_type"] = para["dterm_filter_type"]
        para_used.create_item(
            "dterm_lpf_hz_avg",
            str(
                get_fcut_from_exp(
                    para.get_list("dterm_lpf_dyn_hz")[0],
                    para.get_list("dterm_lpf_dyn_hz")[1],
                    para.get_int("dterm_lpf_dyn_expo"),
                    throttle_avg,
                )
            ),
        )
        para_used.create_item("dterm_lpf_throttle_avg", str(throttle_avg))

        Gd = (
            Gd
            * get_filter(
                filter_types[para.get_int("dterm_filter_type")],
                para_used.get_float("dterm_lpf_hz_avg"),
                Ts,
            )[0]
        )

    if para.get_int("dterm_lpf2_hz") > 0:
        para_used["dterm_lpf2_hz"] = para["dterm_lpf2_hz"]
        para_used["dterm_filter2_type"] = para["dterm_filter2_type"]
        Gd = (
            Gd
            * get_filter(
                filter_types[para.get_int("dterm_filter2_type")],
                para.get_int("dterm_lpf2_hz"),
                Ts,
            )[0]
        )

    if para.get_int("dterm_notch_hz") > 0:
        para_used["dterm_notch_hz"] = para["dterm_notch_hz"]
        para_used["dterm_notch_cutoff"] = para["dterm_notch_cutoff"]
        Gd = (
            Gd
            * get_filter(
                "notch",
                [para.get_int("dterm_notch_cutoff"), para.get_int("dterm_notch_hz")],
                Ts,
            )[0]
        )

    if "dterm_llc_phase" in para.data:
        if para.get_int("dterm_llc_phase") != 0:
            para_used["dterm_llc_freq_hz"] = para["dterm_llc_freq_hz"]
            para_used["dterm_llc_phase"] = para["dterm_llc_phase"]
            Gd = (
                Gd
                * get_filter(
                    "phaseComp",
                    [
                        para.get_int("dterm_llc_freq_hz"),
                        para.get_int("dterm_llc_phase"),
                    ],
                    Ts,
                )[0]
            )

    # P-term filters
    Gf_p = ct.ss(ct.tf(1, 1, Ts))
    if "pterm_llc_phase" in para.data:
        if para.get_int("pterm_llc_phase") != 0:
            para_used["pterm_llc_freq_hz"] = para["pterm_llc_freq_hz"]
            para_used["pterm_llc_phase"] = para["pterm_llc_phase"]
            Gf_p = (
                Gf_p
                * get_filter(
                    "phaseComp",
                    [
                        para.get_int("pterm_llc_freq_hz"),
                        para.get_int("pterm_llc_phase"),
                    ],
                    Ts,
                )[0]
            )

    # Yaw-specific P-term filter
    if ind_ax == 2 and para.get_int("yaw_lpf_hz") > 0:
        para_used["yaw_lpf_hz"] = para["yaw_lpf_hz"]
        Gf_p = Gf_p * get_filter("pt1", para.get_int("yaw_lpf_hz"), Ts)[0]

    # PID gains extraction and validation
    pid_axis = ["rollPID", "pitchPID", "yawPID"]
    if len(para.get_list(pid_axis[ind_ax])) == 5:
        if (
            para.get_list(pid_axis[ind_ax])[2] != para.get_list(pid_axis[ind_ax])[3]
            and para.get_list(pid_axis[ind_ax])[3] != 0
        ):
            warnings.warn(f"{para.get_list(pid_axis[ind_ax])} different D gains")
        # Remove dynamic D-term gain
        para[pid_axis[ind_ax]] = str(
            [para.get_list(pid_axis[ind_ax])[i] for i in [0, 1, 2, 4]]
        ).strip("[]")

    if para.get_list(pid_axis[ind_ax])[3] != 0:
        warnings.warn(f"{pid_axis[ind_ax]} FF is not zero!")

    # Scale PID gains to internal units
    pid_scales = get_pid_scale(ind_ax)
    pid_scales = np.append(pid_scales, 0)  # Add FF scale (unused)
    PID = np.array(para.get_list(pid_axis[ind_ax])) * pid_scales

    Cpi, Cd = calculate_controllers(PID, Gf_p, Ts)
    Cd = Cd * Gd

    return Cpi, Cd, Gf, PID, para_used


def downsample_frd(G, Ts_out, freq_hz):
    """Evaluate frequency response at specified frequencies, return new FRD with Ts_out."""
    freq_hz = np.asarray(freq_hz, dtype=float).ravel()
    omega = 2 * np.pi * freq_hz
    omega_nz = omega[1:]  # Skip DC

    is_discrete = getattr(G, "dt", None) not in (None, 0, 0.0, False)

    if is_discrete:
        z = np.exp(1j * omega_nz * float(G.dt))
        resp_nz = np.stack([np.asarray(ct.evalfr(G, zk)) for zk in z], axis=-1)
    else:
        s = 1j * omega_nz
        resp_nz = np.stack([np.asarray(ct.evalfr(G, sk)) for sk in s], axis=-1)

    resp = np.concatenate([resp_nz[..., 0:1], resp_nz], axis=-1)

    dt_out = None if Ts_out is None else float(Ts_out)
    return ct.FRD(resp, omega, dt=dt_out)


def estimate_frequency_response(
    inp: NDArray[np.floating],
    out: NDArray[np.floating],
    window: NDArray[np.floating],
    Noverlap: int,
    Nest: int,
    Ts: float,
    delta: float = 0.0,
) -> tuple[ct.FRD, ct.FRD, NDArray[np.floating], np.ndarray]:
    """
    Welch method: estimate G(f) = Syu/Suu and coherence from input/output time series.
    Returns amplitude-calibrated single-sided spectra (matches RMS values).
    """
    assert Nest % 2 == 0, "This implementation assumes even Nest."
    assert len(window) == Nest, "window length must equal Nest."

    # Remove DC bias
    inp = inp - np.mean(inp)
    out = out - np.mean(out)

    Ndata = len(inp)
    fs = 1 / Ts
    freq = np.arange(Nest // 2 + 1) * (fs / Nest)
    Nfreq = len(freq)

    # Normalization for amplitude-preserving single-sided spectra
    W = np.sum(window) / Nest / 2

    Pavg = np.zeros((Nfreq, 3), dtype=complex)  # [Suu, Syu, Syy]

    Navg = 0
    ind_start = 0
    ind_end = Nest
    Ndelta = Nest - Noverlap

    while ind_end <= Ndata:
        ind = slice(ind_start, ind_end)

        inp_act = inp[ind]
        out_act = out[ind]

        # Per-segment DC removal
        inp_act = inp_act - np.mean(inp_act)
        out_act = out_act - np.mean(out_act)

        inp_act = window * inp_act
        out_act = window * out_act

        U = np.fft.fft(inp_act) / (Nest * W)
        Y = np.fft.fft(out_act) / (Nest * W)

        # Two-sided cross/auto spectra
        Pact = np.column_stack([U * np.conj(U), Y * np.conj(U), Y * np.conj(Y)])

        # Convert to single-sided (DC and Nyquist get 1/4 power)
        Pseg = Pact[:Nfreq, :]
        Pseg[0, :] = Pseg[0, :] / 4
        Pseg[-1, :] = Pseg[-1, :] / 4

        Pavg += Pseg
        Navg += 1

        ind_start += Ndelta
        ind_end += Ndelta

    if Navg > 0:
        Pavg /= Navg

    g, c = _calc_freqresp_and_cohere(Pavg, delta)

    omega = freq * 2 * np.pi
    G = ct.frd(g, omega)
    C = ct.frd(c, omega)

    return G, C, freq, Pavg


def _calc_freqresp_and_cohere(
    P: np.ndarray, delta: float
) -> tuple[NDArray[np.floating], NDArray[np.floating]]:
    """Compute frequency response G and coherence from cross/auto spectra."""
    Suu = P[:, 0] + delta
    Syu = P[:, 1]
    Syy = P[:, 2]

    g = Syu / Suu
    c = np.abs(Syu) ** 2 / (Suu * Syy)

    return g, np.real(c)


def estimate_spectra(
    inp: NDArray[np.floating],
    window: NDArray[np.floating],
    noverlap: float,
    nest: float,
    ts: float,
) -> tuple[NDArray[np.floating], NDArray[np.floating]]:
    """Welch auto-spectra estimation for multiple input signals."""
    inp = np.array(inp)
    window = np.array(window).flatten()

    if inp.ndim == 1:
        inp = inp[:, np.newaxis]

    assert nest % 2 == 0, "This implementation assumes even nest."
    assert len(window) == nest, "window length must equal nest."

    inp = inp - np.mean(inp, axis=0)

    Ndata, Nsignals = inp.shape
    fs = 1.0 / ts
    freq = np.arange(nest // 2 + 1) * (fs / nest)
    Nfreq = len(freq)

    W = np.sum(window) / nest / 2.0

    Pavg = np.zeros((Nfreq, Nsignals))

    for i in range(Nsignals):
        Navg = 0
        ind_start = 0
        ind_end = nest
        Ndelta = nest - noverlap

        while ind_end <= Ndata:
            inp_act = inp[ind_start:ind_end, i]
            inp_act = inp_act - np.mean(inp_act)
            inp_act = window * inp_act

            U = np.fft.fft(inp_act) / (nest * W)
            Pact = (U * np.conj(U)).real

            Pseg = Pact[0:Nfreq].copy()
            Pseg[0] = Pseg[0] / 4.0
            Pseg[-1] = Pseg[-1] / 4.0

            Pavg[:, i] += Pseg
            Navg += 1

            ind_start += Ndelta
            ind_end += Ndelta

        if Navg > 0:
            Pavg[:, i] /= Navg
    return Pavg, freq


def estimate_spectrogram(
    inp: NDArray[np.floating],
    y: NDArray[np.floating],
    window: NDArray[np.floating],
    noverlap: int,
    nest: int,
    nres: int,
    ts: float,
) -> tuple[NDArray[np.floating], NDArray[np.floating], NDArray[np.floating]]:
    """
    2D spectrogram: frequency vs. auxiliary variable y (e.g., RPM, throttle).
    Returns smoothed power map.
    """
    assert nest % 2 == 0, "This implementation assumes even nest."
    assert len(window) == nest, "window length must equal nest."

    ndata = len(inp)

    # Bin y values into nres rows
    y_min = np.min(y)
    y_max = np.max(y)
    dy = (y_max - y_min) / nres
    y_axis = np.arange(y_min, y_max, dy)

    fs = 1.0 / ts
    freq = np.arange(nest // 2 + 1) * (fs / nest)
    nfreq = len(freq)

    w_norm = np.sum(window) / nest / 2

    pavg = np.zeros((nres, nfreq))
    navg = np.zeros(nres)

    ind_start = 0
    ind_end = nest
    ndelta = nest - noverlap

    while ind_end <= ndata:
        seg_idx = slice(ind_start, ind_end)
        inp_act = inp[seg_idx].copy()
        inp_act = window * inp_act

        u = np.fft.fft(inp_act) / (nest * w_norm)
        pact = np.real(u * np.conj(u))

        pseg = pact[:nfreq].copy()
        pseg[0] = pseg[0] / 4
        pseg[-1] = pseg[-1] / 4

        # Map segment's y values to spectrogram rows
        y_seg = y[seg_idx]
        ind_y = np.round(
            (y_seg - y_min) / max(y_max - y_min, np.finfo(float).eps) * (nres - 1)
        ).astype(int)
        ind_y = np.clip(ind_y, 0, nres - 1)

        unique_inds, counts = np.unique(ind_y, return_counts=True)

        for row_idx, count in zip(unique_inds, counts):
            pavg[row_idx, :] += count * pseg
            navg[row_idx] += count

        ind_start += ndelta
        ind_end += ndelta

    # Average per row
    nonzero = navg != 0
    pavg[nonzero, :] = pavg[nonzero, :] / navg[nonzero, np.newaxis]

    pavg = _smooth2d(pavg)

    return pavg, freq, y_axis


def _smooth2d(pavg: NDArray[np.floating]) -> NDArray[np.floating]:
    """2D smoothing with weighted 3x3 kernel."""
    kernel = np.array([[1, 3, 1], [3, 5, 3], [1, 3, 1]], dtype=float)
    kernel = kernel / kernel.sum()

    num = convolve2d(pavg, kernel, mode="same", boundary="fill")
    den = convolve2d(np.ones_like(pavg), kernel, mode="same", boundary="fill")

    return num / den


def get_fcut_from_D_and_fcenter(D: float, fcent: float) -> float:
    """Convert damping ratio D and center frequency to cutoff frequency."""
    Q = 1 / 2 / D
    fcut = fcent / 2 / Q * (-1 + np.sqrt(1 + 4 * Q**2))
    return fcut


def get_fcut_from_exp(
    dynLpfMin: float,
    dynLpfMax: float,
    expo: float,
    throttle: float,
) -> float:
    """Calculate dynamic filter cutoff from throttle position with exponential curve."""
    expof = expo / 10.0
    curve = throttle * (1 - throttle) * expof + throttle
    fcut = (dynLpfMax - dynLpfMin) * curve + dynLpfMin
    return fcut


def get_filter(
    filter_type: str, f_cut: Union[float, List[float], np.ndarray], Ts: float
):
    """
    Build discrete-time filter transfer function.
    Returns state-space, numerator, and denominator coefficients.
    """
    if isinstance(f_cut, (list, tuple)):
        f_cut = np.array(f_cut)

    G = ct.tf([1], [1], Ts)

    if filter_type == "pt1":
        if isinstance(f_cut, np.ndarray) and f_cut.size == 1:
            fc = float(f_cut.item())
        elif isinstance(f_cut, (int, float)):
            fc = float(f_cut)
        else:
            fc = f_cut  # type: ignore

        RC = 1.0 / (2.0 * np.pi * fc)
        k = Ts / (RC + Ts)
        G = ct.tf([k, 0], [1, (k - 1)], Ts)

    elif filter_type == "pt2":
        order = 2.0
        orderCutoffCorrection = 1.0 / np.sqrt(2 ** (1 / order) - 1)
        f_corrected = f_cut * orderCutoffCorrection
        RC = 1.0 / (2.0 * np.pi * f_corrected)
        k = Ts / (RC + Ts)
        G = ct.tf([k**2, 0, 0], [1, 2 * (k - 1), (k - 1) ** 2], Ts)

    elif filter_type == "pt3":
        order = 3.0
        orderCutoffCorrection = 1.0 / np.sqrt(2 ** (1 / order) - 1)
        f_corrected = f_cut * orderCutoffCorrection
        RC = 1.0 / (2.0 * np.pi * f_corrected)
        k = Ts / (RC + Ts)
        G = ct.tf([k**3, 0, 0, 0], [1, 3 * (k - 1), 3 * (k - 1) ** 2, (k - 1) ** 3], Ts)

    elif filter_type == "biquad":
        Q = 1.0 / np.sqrt(2)
        omega = 2.0 * np.pi * f_cut * Ts
        sn = np.sin(omega)
        cs = np.cos(omega)
        alpha = sn / (2.0 * Q)

        b1 = (1.0 - cs) / (1.0 + alpha)
        b0 = b1 * 0.5
        b2 = b0
        a1 = -2.0 * cs / (1.0 + alpha)
        a2 = (1.0 - alpha) / (1.0 + alpha)

        G = ct.tf([b0, b1, b2], [1, a1, a2], Ts)

    elif filter_type == "notch":
        if isinstance(f_cut, (float, int)):
            raise ValueError("f_cut must be a list or array for notch filter")

        Q = get_notch_Q(f_cut[1], f_cut[0])
        omega = 2.0 * np.pi * f_cut[1] * Ts
        sn = np.sin(omega)
        cs = np.cos(omega)
        alpha = sn / (2.0 * Q)

        b0 = 1.0 / (1.0 + alpha)
        b1 = -2.0 * cs / (1.0 + alpha)
        b2 = b0
        a1 = b1
        a2 = (1.0 - alpha) / (1.0 + alpha)

        G = ct.tf([b0, b1, b2], [1, a1, a2], Ts)

    elif filter_type == "phaseComp":
        """Linear lead-lag compensator for phase adjustment."""
        if isinstance(f_cut, (float, int)):
            raise ValueError("f_cut must be a list or array for phaseComp")

        centerFreqHz = f_cut[0]
        centerPhaseDeg = f_cut[1]
        omega = 2.0 * np.pi * centerFreqHz * Ts
        sn = np.sin(centerPhaseDeg * np.pi / 180.0)
        gain = (1.0 + sn) / (1.0 - sn)

        # Approximate prewarping
        alpha = (12.0 - omega * omega) / (6.0 * omega * np.sqrt(gain))

        b0_val = 1.0 + alpha * gain
        b1_val = 2.0 - b0_val
        a1_val = 1.0 - alpha
        a0_val = 1.0 / (1.0 + alpha)

        b0 = b0_val * a0_val
        b1 = b1_val * a0_val
        a1 = a1_val * a0_val

        G = ct.tf([b0, b1], [1, a1], Ts)

    elif filter_type == "leadlag1":
        if isinstance(f_cut, (float, int)):
            raise ValueError("f_cut must be a list or array for leadlag1")

        fz = f_cut[0]
        fp = f_cut[1]
        alpha_ll = fz / fp
        centerFreqHz = fp * np.sqrt(alpha_ll)
        centerPhaseDeg = 180.0 / np.pi * np.arcsin((1.0 - alpha_ll) / (1.0 + alpha_ll))

        return get_filter("phaseComp", [centerFreqHz, centerPhaseDeg], Ts)

    else:
        raise ValueError(f"filter_type '{filter_type}' not valid")

    B = np.array(G.num[0][0])  # type: ignore
    A = np.array(G.den[0][0])  # type: ignore
    G_ss = ct.ss(G)

    return G_ss, B, A


def get_notch_Q(centerFreq: float, cutoffFreq: float) -> float:
    """Calculate notch filter Q factor from center and cutoff frequencies."""
    Q = (centerFreq * cutoffFreq) / (
        (centerFreq * centerFreq) - (cutoffFreq * cutoffFreq)
    )
    return Q


def get_pid_scale(ind_ax: int) -> list[float]:
    """Betaflight internal PID gain scaling factors (yaw integrator is 2.5x)."""
    PTERM_SCALE = 0.032029
    ITERM_SCALE = 0.244381
    DTERM_SCALE = 0.000529

    pid_scale = [PTERM_SCALE, ITERM_SCALE, DTERM_SCALE]

    if ind_ax == 2:  # Yaw axis
        pid_scale[1] *= 2.5

    return pid_scale
