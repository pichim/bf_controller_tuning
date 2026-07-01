"""
==========================================================================
PLOT_UTILS - Betaflight Controller Analysis Visualization Class
==========================================================================
Purpose:
    This class provides visualization methods for analyzing quadcopter
    flight controller performance and tuning data.

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
import matplotlib.pyplot as plt
import control as ct

from typing import List, Dict
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LogNorm


class PlotUtils:
    """Visualization utilities for flight analysis."""

    def __init__(self, legend: bool = True):
        """
        Initialize plot utilities.

        Parameters
        ----------
        legend : bool
            Whether to show legends on plots
        """

        self.do_insert_legends = legend
        self.linewidth = 1.2

        self.axis_names = ['Roll', 'Pitch', 'Yaw']

        plt.style.use('seaborn-v0_8-darkgrid')

        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['lines.linewidth'] = self.linewidth

    # ======================================================================
    # Figure Flight Data
    # ======================================================================

    def plot_flight_data(
        self,
        df,
        groups: List[Dict],
        fig_title: str = ''
    ):
        """
        Plot selected flight data signals.

        Parameters
        ----------
        df : FlightData
            Flight data object
        groups : List[Dict]
            List of subplot specifications
        fig_title : str
            Figure title
        """

        n_subplots = len(groups)

        fig, axes = plt.subplots(
            n_subplots,
            1,
            figsize=(8, 6)
        )

        fig.canvas.manager.set_window_title(fig_title)

        if n_subplots == 1:
            axes = [axes]

        if fig_title:
            fig.suptitle(
                fig_title,
                fontsize=14,
                fontweight='bold'
            )

        for i, (ax, group) in enumerate(zip(axes, groups)):

            idx_list = group['idx']

            # --------------------------------------------------------------
            # Legend Entries
            # --------------------------------------------------------------

            if 'legend' in group:
                legend_entries = group['legend']
            else:
                legend_entries = [
                    f'Signal {k + 1}'
                    for k in range(len(idx_list))
                ]

            # --------------------------------------------------------------
            # Plot Data
            # --------------------------------------------------------------

            for k, idx in enumerate(idx_list):

                ax.plot(
                    df.time,
                    df.data[:, idx],
                    linewidth=self.linewidth,
                    label=legend_entries[k]
                )

            ax.grid(True)

            ax.set_xlim([
                df.time[0],
                df.time[-1]
            ])

            ax.set_ylabel(group['ylabel'])

            if self.do_insert_legends:
                ax.legend(loc='best')

            if i == n_subplots - 1:
                ax.set_xlabel('Time [s]')

        plt.tight_layout()

    # ======================================================================
    # Figure Evaluation Time
    # ======================================================================

    def plot_eval_time(
        self,
        time: np.ndarray
    ):
        """
        Plot evaluation time / logging sample time.
        """

        delta_time_us = np.diff(time) * 1.0e6

        fig, ax = plt.subplots(figsize=(8, 6))

        fig.canvas.manager.set_window_title(
            'Evaluation Time Flight'
        )

        ax.plot(
            time[:-1],
            delta_time_us,
            linewidth=self.linewidth
        )

        mean_val = np.mean(delta_time_us)

        ax.axhline(
            mean_val,
            color='k',
            linestyle='--',
            linewidth=self.linewidth
        )

        ax.text(
            time[0],
            mean_val,
            'Mean',
            ha='left',
            va='bottom',
            fontsize=9,
            color='k'
        )

        ax.grid(True)

        ax.set_title(
            f'Mean: {np.mean(delta_time_us):.2f} us, '
            f'Median: {np.median(delta_time_us):.2f} us, '
            f'Std: {np.std(delta_time_us):.2f} us'
        )

        ax.set_xlabel('Time (sec)')
        ax.set_ylabel('Ts log (us)')

        ax.set_xlim([0, time[-1]])

        plt.tight_layout()
    
        # ======================================================================
    # Figure Gyro Spectra
    # ======================================================================

    def plot_gyro_spectra(self, fa):
        """
        Plot gyro spectra.

        Parameters
        ----------
        fa : FlightAnalyzer
            Flight analyzer object
        """

        fig, axes = plt.subplots(
            3,
            1,
            figsize=(8, 6)
        )

        fig.canvas.manager.set_window_title(
            'Gyro Spectra'
        )

        # --------------------------------------------------------------
        # Unfiltered Gyro Spectra
        # --------------------------------------------------------------

        axes[0].plot(
            fa.freq_spectra,
            fa.spectra[:, 0:3],
            linewidth=self.linewidth
        )

        axes[0].grid(
            True,
            which='both',
            alpha=0.3
        )

        axes[0].set_ylabel('Gyro (deg/s)')
        axes[0].set_yscale('log')

        axes[0].set_title(
            'Unfiltered gyro magnitude spectra'
        )

        axes[0].legend(
            ['Roll', 'Pitch', 'Yaw'],
            loc='upper right'
        )

        # --------------------------------------------------------------
        # Filtered Gyro Spectra
        # --------------------------------------------------------------

        axes[1].plot(
            fa.freq_spectra,
            fa.spectra[:, 3:6],
            linewidth=self.linewidth
        )

        axes[1].grid(
            True,
            which='both',
            alpha=0.3
        )

        axes[1].set_ylabel('Gyro (deg/s)')
        axes[1].set_yscale('log')

        axes[1].set_title(
            'Filtered (ADC) gyro magnitude spectra'
        )

        if self.do_insert_legends:

            axes[1].legend(
                ['Roll', 'Pitch', 'Yaw'],
                loc='upper right'
            )

        # --------------------------------------------------------------
        # Axis Sum Spectra
        # --------------------------------------------------------------

        axes[2].plot(
            fa.freq_spectra,
            fa.spectra[:, 6:9],
            linewidth=self.linewidth
        )

        axes[2].grid(
            True,
            which='both',
            alpha=0.3
        )

        axes[2].set_ylabel('AxisSum')
        axes[2].set_xlabel('Frequency (Hz)')

        axes[2].set_yscale('log')

        axes[2].set_title(
            'Axis sum spectra'
        )

        axes[2].legend(
            ['Roll', 'Pitch', 'Yaw'],
            loc='upper right'
        )

        # --------------------------------------------------------------
        # Axis Limits
        # --------------------------------------------------------------

        nyq = 1 / (2 * fa.Ts_log)

        for ax in axes:

            ax.set_xlim([0, nyq])

            try:
                ax.autoscale(
                    enable=True,
                    axis='y',
                    tight=False
                )

            except:
                pass

        plt.tight_layout()

    # ======================================================================
    # Figure Spectrogram
    # ======================================================================

    def plot_spectrogram(
        self,
        fa,
        num_spectrograms: int = 3
    ):
        """
        Plot gyro spectrograms.

        Parameters
        ----------
        fa : FlightAnalyzer
            Flight analyzer object
        num_spectrograms : int
            Number of spectrograms
        """

        fig = plt.figure(figsize=(8, 6))

        fig.canvas.manager.set_window_title(
            'Gyro Spectrograms'
        )

        axes_labels = ['Roll', 'Pitch', 'Yaw']

        c_lim = [5e-2, 3e0]

        # --------------------------------------------------------------
        # Unfiltered Spectrograms
        # --------------------------------------------------------------

        for i in range(num_spectrograms):

            ax = plt.subplot(2, 3, i + 1)

            im = ax.pcolormesh(
                fa.freq_spectrogram[i],
                fa.throttle_all[i],
                fa.spectrogram_unf[i],
                shading='auto',
                cmap='jet',
                norm=LogNorm(
                    vmin=c_lim[0],
                    vmax=c_lim[1]
                )
            )

            if i == 0:
                ax.set_ylabel('Throttle (%)')

            ax.set_xlabel('Frequency (Hz)')

            ax.set_title(
                f'{axes_labels[i]} – without filter'
            )

            ax.set_ylim([0, 100])

            plt.colorbar(im, ax=ax)

        # --------------------------------------------------------------
        # Filtered Spectrograms
        # --------------------------------------------------------------

        for i in range(num_spectrograms):

            ax = plt.subplot(2, 3, i + 4)

            im = ax.pcolormesh(
                fa.freq_spectrogram[i],
                fa.throttle_all[i],
                fa.spectrogram_fil[i],
                shading='auto',
                cmap='jet',
                norm=LogNorm(
                    vmin=c_lim[0],
                    vmax=c_lim[1]
                )
            )

            if i == 0:
                ax.set_ylabel('Throttle (%)')

            ax.set_xlabel('Frequency (Hz)')

            ax.set_title(
                f'{axes_labels[i]} – with filter'
            )

            ax.set_ylim([0, 100])

            plt.colorbar(im, ax=ax)

        plt.suptitle(
            'Gyro Spectrograms',
            fontsize=14,
            fontweight='bold'
        )

        plt.tight_layout()
    
        # ======================================================================
    # Figure Bode Plot Plant Coherence
    # ======================================================================

    def plot_bode_plant(
        self,
        td,
        ind_ax: int,
        custom_label: str = 'Signal',
        G: str = 'Plant'
    ):
        """
        Plot bode plot and coherence.

        Parameters
        ----------
        td : tuning object
            Tuning data object
        ind_ax : int
            Axis index
        custom_label : str
            Label for plot title
        G : str
            'Plant',
            'Complementary Sensitivity'
            or 'Controller'
        """

        axis_name = self.axis_names[ind_ax]

        fig_name = (
            f'Bode Plot {G} '
            f'{custom_label} - '
            f'{axis_name}'
        )

        plant_title = (
            f'{G} ({custom_label}) - '
            f'{axis_name}'
        )

        fig = plt.figure(figsize=(8, 6))

        fig.canvas.manager.set_window_title(fig_name)

        gs = GridSpec(
            3,
            1,
            height_ratios=[2, 2, 1]
        )

        # --------------------------------------------------------------
        # Create Axes
        # --------------------------------------------------------------

        ax1 = fig.add_subplot(gs[0])

        ax2 = fig.add_subplot(
            gs[1],
            sharex=ax1
        )

        ax3 = fig.add_subplot(
            gs[2],
            sharex=ax1
        )

        # --------------------------------------------------------------
        # Select System
        # --------------------------------------------------------------

        if G == 'Plant':

            sys = td.P[ind_ax]
            coh = td.Coh_P[ind_ax]

        elif G == 'Complementary Sensitivity':

            sys = td.T[ind_ax]
            coh = td.Coh_T[ind_ax]

        elif G == 'Controller':

            sys = td.C[ind_ax]
            coh = td.Coh_C[ind_ax]

        else:

            raise ValueError(
                "G must be "
                "'Plant', "
                "'Complementary Sensitivity' "
                "or 'Controller'"
            )

        # --------------------------------------------------------------
        # Calculate Bode
        # --------------------------------------------------------------

        mag, phase, omega = ct.bode(
            sys,
            plot=False
        )

        freq_hz = (
            omega
            / (2 * np.pi)
        )

        mag_db = 20 * np.log10(
            np.squeeze(mag)
        )

        phase_deg = (
            np.squeeze(phase)
            * 180
            / np.pi
        )

        phase_wrapped = (
            np.mod(phase_deg + 180, 360)
            - 180
        )

        # --------------------------------------------------------------
        # Magnitude Plot
        # --------------------------------------------------------------

        ax1.semilogx(
            freq_hz,
            mag_db,
            'k',
            linewidth=self.linewidth
        )

        ax1.grid(
            True,
            which='both',
            alpha=0.3
        )

        ax1.set_ylabel('Magnitude (dB)')

        ax1.set_title(plant_title)

        # --------------------------------------------------------------
        # Phase Plot
        # --------------------------------------------------------------

        ax2.semilogx(
            freq_hz,
            phase_wrapped,
            'k',
            linewidth=self.linewidth
        )

        ax2.grid(
            True,
            which='both',
            alpha=0.3
        )

        ax2.set_ylabel('Phase (deg)')

        # --------------------------------------------------------------
        # Coherence Plot
        # --------------------------------------------------------------

        coh_data = np.abs(
            np.squeeze(coh.fresp)
        )

        coh_freq = (
            coh.omega
            / (2 * np.pi)
        )

        ax3.semilogx(
            coh_freq,
            coh_data,
            'k',
            linewidth=self.linewidth
        )

        ax3.grid(
            True,
            which='both',
            alpha=0.3
        )

        ax3.set_ylabel('Coherence')

        ax3.set_ylim([0, 1.1])

        ax3.set_xlabel('Frequency (Hz)')

        plt.tight_layout()

    # ======================================================================
    # Figure Gang of Four
    # ======================================================================

    def plot_gang_of_four(
        self,
        td,
        label: str = 'Signal'
    ):
        """
        Plot Gang of Four.

        Parameters
        ----------
        td : tuning object
            Tuning data object
        label : str
            Plot label
        """

        ind_ax = td.ind_ax

        axis_name = self.axis_names[ind_ax]

        fig, axes = plt.subplots(
            2,
            2,
            figsize=(8, 6)
        )

        fig.canvas.manager.set_window_title(
            f'Gang of Four '
            f'{label} - '
            f'{axis_name}'
        )

        fig.suptitle(
            f'Gang of Four '
            f'{label} - '
            f'{axis_name}',
            fontsize=14,
            fontweight='bold'
        )

        # --------------------------------------------------------------
        # Helper Function
        # --------------------------------------------------------------

        def plot_frd(
            ax,
            systems,
            title,
            xlabel=None
        ):

            for sys, style, lbl in systems:

                mag, _, omega = ct.bode(
                    sys,
                    plot=False
                )

                freq_hz = (
                    omega
                    / (2 * np.pi)
                )

                mag_db = 20 * np.log10(
                    np.squeeze(mag)
                )

                ax.semilogx(
                    freq_hz,
                    mag_db,
                    style,
                    linewidth=self.linewidth,
                    label=lbl
                )

            ax.grid(
                True,
                which='both',
                alpha=0.3
            )

            ax.set_ylabel('Magnitude (dB)')

            ax.set_title(title)

            if xlabel:
                ax.set_xlabel(xlabel)

            if self.do_insert_legends:
                ax.legend(loc='best')

        # --------------------------------------------------------------
        # Tracking T
        # --------------------------------------------------------------

        plot_frd(
            axes[0, 0],
            [
                (
                    td.CL_ana.T,
                    '--',
                    'actual'
                ),
                (
                    td.CL_ana_new.T,
                    '-',
                    'new'
                ),
                (
                    td.T[ind_ax],
                    ':',
                    'measured'
                ),
            ],
            title='Tracking T'
        )

        # --------------------------------------------------------------
        # Sensitivity S
        # --------------------------------------------------------------

        plot_frd(
            axes[0, 1],
            [
                (
                    td.CL_ana.S,
                    '--',
                    'actual'
                ),
                (
                    td.CL_ana_new.S,
                    '-',
                    'new'
                ),
            ],
            title='Sensitivity S'
        )

        # --------------------------------------------------------------
        # Controller Effort SC
        # --------------------------------------------------------------

        plot_frd(
            axes[1, 0],
            [
                (
                    td.CL_ana.SC,
                    '--',
                    'actual'
                ),
                (
                    td.CL_ana_new.SC,
                    '-',
                    'new'
                ),
            ],
            title='Controller Effort SC',
            xlabel='Frequency (Hz)'
        )

        # --------------------------------------------------------------
        # Compliance SP
        # --------------------------------------------------------------

        plot_frd(
            axes[1, 1],
            [
                (
                    td.CL_ana.SP,
                    '--',
                    'actual'
                ),
                (
                    td.CL_ana_new.SP,
                    '-',
                    'new'
                ),
            ],
            title='Compliance SP',
            xlabel='Frequency (Hz)'
        )

        plt.tight_layout()

    
        # ======================================================================
    # Figure Step Compliance
    # ======================================================================

    def plot_step_compliance(
        self,
        td,
        label: str = 'Signal',
        ylab: str = 'Response'
    ):
        """
        Plot tracking step response and compliance response.

        Parameters
        ----------
        td : tuning object
            Tuning data object
        label : str
            Plot label
        ylab : str
            Y-axis label
        """

        axis_name = self.axis_names[td.ind_ax]

        fig, axes = plt.subplots(
            2,
            1,
            figsize=(8, 6)
        )

        fig.canvas.manager.set_window_title(
            f'Step Response {label} - {axis_name}'
        )

        fig.suptitle(
            f'Step Response {label} - {axis_name}',
            fontsize=14,
            fontweight='bold'
        )

        # --------------------------------------------------------------
        # Tracking
        # --------------------------------------------------------------

        axes[0].plot(
            td.step_time,
            td.step_resp_tra,
            linewidth=self.linewidth
        )

        axes[0].grid(True)

        axes[0].set_ylabel(ylab)

        axes[0].set_title(
            f'Tracking T - {axis_name}'
        )

        if self.do_insert_legends:

            axes[0].legend(
                [
                    'actual calculated',
                    'new calculated',
                    'measured'
                ],
                loc='best'
            )

        # --------------------------------------------------------------
        # Compliance
        # --------------------------------------------------------------

        axes[1].plot(
            td.step_time,
            td.step_resp_com,
            linewidth=self.linewidth
        )

        axes[1].grid(True)

        axes[1].set_ylabel(ylab)
        axes[1].set_xlabel('Time (sec)')

        axes[1].set_title('Compliance SP')

        if self.do_insert_legends:

            axes[1].legend(
                [
                    'actual calculated',
                    'new calculated'
                ],
                loc='best'
            )

        axes[0].set_xlim([0, 0.3])
        axes[1].set_xlim([0, 0.3])

        plt.tight_layout()

    # ======================================================================
    # Figure Step Response
    # ======================================================================

    def plot_step_response(
        self,
        td,
        label: str = 'Signal',
        ylab: str = 'Response'
    ):
        """
        Plot only tracking step response.

        Parameters
        ----------
        td : tuning object
            Tuning data object
        label : str
            Plot label
        ylab : str
            Y-axis label
        """

        axis_name = self.axis_names[td.ind_ax]

        fig, ax = plt.subplots(figsize=(8, 6))

        fig.canvas.manager.set_window_title(
            f'Step Response {label} - {axis_name}'
        )

        ax.plot(
            td.step_time,
            td.step_resp_tra,
            linewidth=self.linewidth
        )

        ax.grid(True)

        ax.set_ylabel(ylab)
        ax.set_xlabel('Time (sec)')

        ax.set_xlim([0, 1])

        ax.set_title(
            f'Tracking T - {axis_name}'
        )

        if self.do_insert_legends:

            ax.legend(
                [
                    'actual calculated',
                    'new calculated',
                    'measured'
                ],
                loc='best'
            )

        plt.tight_layout()