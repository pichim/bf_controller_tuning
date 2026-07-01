"""
==========================================================================
FLIGHT ANALYZER - Betaflight Controller Analysis ANALYSIS CLASS
==========================================================================

Purpose: 
    Get Spectra and Spectrogram out of Data

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
from typing import Optional, List
from scipy.signal import hann

from .pidtuninglib import estimate_spectra, estimate_spectrogram


class FlightAnalyzer:
    """Flight data spectral analyzer."""
    
    def __init__(self, data: np.ndarray, ind, Ts_log: float):
        """
        Initialize flight analyzer.
        
        Parameters
        ----------
        data : np.ndarray
            Flight data array
        ind : ColumnIndices
            Column indices object
        Ts_log : float
            Logging sample time in seconds
        """
        self.data = data
        self.ind = ind
        self.Ts_log = Ts_log
        self.linewidth = 1.2
        
        # Storage for results
        self.freq_spectra: Optional[np.ndarray] = None
        self.spectra: Optional[np.ndarray] = None
        self.freq_spectrogram: List[np.ndarray] = []
        self.spectrogram_unf: List[np.ndarray] = []
        self.spectrogram_fil: List[np.ndarray] = []
        self.throttle_all: List[np.ndarray] = []
    
    def calculate_spectra(
        self,
        Nestfaspec: float,
        koverlapspec: float
    ) -> 'FlightAnalyzer':
        """
        Calculate power spectra of gyro and control signals.
        
        Parameters
        ----------
        Nestfaspec : float
            Window length in seconds
        koverlapspec : float
            Overlap factor (0-1)
        
        Returns
        -------
        FlightAnalyzer
            Self with calculated spectra
        """
        # Select data columns for spectral analysis
        data_for_spectra = self.data[:, np.concatenate([
            self.ind.gyroUnfilt,
            self.ind.gyroADC,
            self.ind.axisSum,
            self.ind.setpoint[:3]
        ])]
        
        # Configure spectral estimation parameters
        Nest_spec = int(np.round(Nestfaspec / self.Ts_log))
        window_spec = hann(Nest_spec, sym=False)  # periodic
        Noverlap = int(np.floor(koverlapspec * Nest_spec))
        
        # Calculate power spectral density
        pxx, self.freq_spectra = estimate_spectra(
            data_for_spectra,
            window_spec,
            Noverlap,
            Nest_spec,
            self.Ts_log
        )
        
        # Convert power to amplitude spectra
        self.spectra = np.sqrt(pxx)
        
        return self
    
    def calculate_spectrogram(
        self,
        Nestfaspec: float,
        koverlapspec: float
    ) -> 'FlightAnalyzer':
        """
        Calculate spectrograms vs throttle.
        
        Parameters
        ----------
        Nestfaspec : float
            Window length in seconds
        koverlapspec : float
            Overlap factor (0-1)
        
        Returns
        -------
        FlightAnalyzer
            Self with calculated spectrograms
        """
        Nest_spec = int(np.round(Nestfaspec / self.Ts_log))
        window_spec = hann(Nest_spec, sym=False)
        Noverlap = int(np.floor(koverlapspec * Nest_spec))
        
        # Calculate throttle resolution
        Nres = int(np.floor(np.max(self.data[:, self.ind.setpoint[3]]) / 1e1 / 2))
        
        # Initialize storage
        num_spectrograms = 3  # One per axis
        self.spectrogram_unf = []
        self.spectrogram_fil = []
        self.freq_spectrogram = []
        self.throttle_all = []
        
        # Calculate spectrograms for unfiltered gyro
        for spectrogram_nr in range(num_spectrograms):
            pxx, freq, throttle = estimate_spectrogram(
                self.data[:, self.ind.gyroUnfilt[spectrogram_nr]],
                self.data[:, self.ind.setpoint[3]] / 10.0,
                window_spec,
                Noverlap,
                Nest_spec,
                Nres,
                self.Ts_log
            )
            
            self.spectrogram_unf.append(np.sqrt(pxx))
            self.freq_spectrogram.append(freq)
            self.throttle_all.append(throttle)
        
        # Calculate spectrograms for filtered gyro
        for spectrogram_nr in range(num_spectrograms):
            pxx, freq, throttle = estimate_spectrogram(
                self.data[:, self.ind.gyroADC[spectrogram_nr]],
                self.data[:, self.ind.setpoint[3]] / 10.0,
                window_spec,
                Noverlap,
                Nest_spec,
                Nres,
                self.Ts_log
            )
            
            self.spectrogram_fil.append(np.sqrt(pxx))
        
        return self