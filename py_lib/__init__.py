
"""
Betaflight Controller Tuning Analysis Library

This package provides tools for analyzing flight controller logs and tuning
PID controllers for quadcopters running Betaflight firmware.

Classes:
    FlightData: Load and preprocess flight log data
    GyroCtrlTuning: Controller analysis and tuning
    FlightAnalyzer: Spectral analysis (spectra, spectrograms)
    PlotUtils: Visualization tools

Modules:
    pidtuninglib: Helper functions for signal processing and control theory
"""

__version__ = '1.0.0'
__author__ = 'Janick Dort, Yuri Bianchi, Dario Jurietti'


from .flight_data import FlightData, ColumnIndices
from .flight_analyzer import FlightAnalyzer
from .gyro_ctrl_tuning import GyroCtrlTuning
from .plot_utils import PlotUtils
from .pidtuninglib import header_info

__all__ = [
    'FlightData',
    'ColumnIndices',
    'FlightAnalyzer',
    'GyroCtrlTuning',
    'PlotUtils',
    'header_info',
]