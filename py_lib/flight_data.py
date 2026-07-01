"""
==========================================================================
FLIGHT DATA - Betaflight Controller Analysis (data import class)
==========================================================================

Purpose:
    Read data for further analysis

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
import pandas as pd
import pickle
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional
import time

from .pidtuninglib import header_info


@dataclass
class ColumnIndices:
    """Storage for column indices from flight log."""

    # File columns
    time: int = 0

    # Raw flight data
    gyroUnfilt: np.ndarray = field(default_factory=lambda: np.array([]))
    gyroADC: np.ndarray = field(default_factory=lambda: np.array([]))
    setpoint: np.ndarray = field(default_factory=lambda: np.array([]))
    rcCommand: np.ndarray = field(default_factory=lambda: np.array([]))

    # Controller signals
    axisP: np.ndarray = field(default_factory=lambda: np.array([]))
    axisI: np.ndarray = field(default_factory=lambda: np.array([]))
    axisD: np.ndarray = field(default_factory=lambda: np.array([]))
    axisSum: np.ndarray = field(default_factory=lambda: np.array([]))
    axisSumPI: np.ndarray = field(default_factory=lambda: np.array([]))

    # Additional data
    motor: np.ndarray = field(default_factory=lambda: np.array([]))
    debug: np.ndarray = field(default_factory=lambda: np.array([]))
    heading: np.ndarray = field(default_factory=lambda: np.array([]))

    # Debug remapping
    sinarg: int = 0
    currentAngle: np.ndarray = field(default_factory=lambda: np.array([]))
    angleTarget: np.ndarray = field(default_factory=lambda: np.array([]))
    angleRate: np.ndarray = field(default_factory=lambda: np.array([]))


class FlightData:
    """
    Flight data loader and preprocessor for Betaflight logs.
    """

    def __init__(self, file_path: str):

        # =============================================================
        # File Handling
        # =============================================================

        self.file_path = Path(file_path)

        # =============================================================
        # Raw Flight Data
        # =============================================================

        self.time: Optional[np.ndarray] = None
        self.data: Optional[np.ndarray] = None
        self.ind: Optional[ColumnIndices] = None
        self.para: Optional[header_info] = None

        # =============================================================
        # Calculated Data
        # =============================================================

        self.Ts_log: Optional[float] = None
        self.Ts_cntr: Optional[float] = None

        # =============================================================
        # Default Values
        # =============================================================

        self.linewidth = 1.2

    def get_data(self) -> "FlightData":
        """
        Load and process flight log data.
        """

        # =============================================================
        # Load and Process Flight Log Data
        # =============================================================

        self.para, nheader, self.ind, ind_cntr = (
            self._extract_header_information()
        )

        # =============================================================
        # Load Data
        # =============================================================
        # Load data from cached pickle file if available.
        # Otherwise, read CSV file and cache the result for faster loading.
        # =============================================================

        print("Loading flight data...")
        start_time = time.time()

        pickle_path = self.file_path.with_suffix(".pkl")

        try:
            with open(pickle_path, "rb") as f:
                self.data = pickle.load(f)

            print(
                f"Loaded from cache in "
                f"{time.time() - start_time:.3f}s"
            )

        except (FileNotFoundError, pickle.PickleError, EOFError):

            self.data = pd.read_csv(
                self.file_path,
                skiprows=nheader,
                header=None,
            ).values

            with open(pickle_path, "wb") as f:
                pickle.dump(self.data, f)

            print(
                f"Loaded from CSV and cached in "
                f"{time.time() - start_time:.3f}s"
            )

        # =============================================================
        # Data Preprocessing
        # =============================================================

        self._preprocess_data(ind_cntr)

        return self

    def _extract_header_information(self):
        """
        Extract header parameters and column indices from log file.
        """

        # =============================================================
        # Extract Header Information
        # =============================================================
        # Reads Betaflight parameters and finds the data header line.
        # The data header line contains the column names.
        # =============================================================

        para = header_info.get_header(self.file_path)

        nheader = 0
        ind = ColumnIndices()
        ind_cntr = 0

        with open(self.file_path, "r") as f:
            for line_num, line in enumerate(f):

                nheader = line_num + 1

                if "loopIteration" in line:
                    ind, ind_cntr = self._parse_column_names(line)
                    break

        return para, nheader, ind, ind_cntr

    def _parse_column_names(self, header_line: str):
        """
        Parse column names from data header line.
        """

        # =============================================================
        # Parse Column Names
        # =============================================================
        # Converts Betaflight column names into index arrays.
        # These indices are later used to access signals directly.
        # =============================================================

        ind = ColumnIndices()

        columns = [
            col.strip().strip('"')
            for col in header_line.strip().split(",")
        ]
        
        gyro_unfilt = []
        gyro_adc = []
        setpoint = []
        rc_command = []

        axis_p = []
        axis_i = []
        axis_d = []
        axis_sum = []

        motor = []
        debug = []
        heading = []

        for idx, col in enumerate(columns):

            if col == "time" or col == "time (us)":
                ind.time = idx

            elif "gyroUnfilt" in col:
                gyro_unfilt.append(idx)

            elif "gyroADC" in col:
                gyro_adc.append(idx)

            elif col.startswith("setpoint["):
                setpoint.append(idx)

            elif col.startswith("rcCommand["):
                rc_command.append(idx)

            elif col.startswith("axisP["):
                axis_p.append(idx)

            elif col.startswith("axisI["):
                axis_i.append(idx)

            elif col.startswith("axisD["):
                axis_d.append(idx)

            elif col.startswith("axisSum["):
                axis_sum.append(idx)

            elif col.startswith("motor["):
                motor.append(idx)

            elif col.startswith("debug["):
                debug.append(idx)

            elif col.startswith("heading["):
                heading.append(idx)

            elif col == "flightModeFlags":
                ind.flightModeFlags = idx

            elif col == "stateFlags":
                ind.stateFlags = idx

            elif col == "failsafePhase":
                ind.failsafePhase = idx

            
        # Store parsed column indices
        ind.gyroUnfilt = np.array(gyro_unfilt)
        ind.gyroADC = np.array(gyro_adc)
        ind.setpoint = np.array(setpoint)
        ind.rcCommand = np.array(rc_command)

        ind.axisP = np.array(axis_p)
        ind.axisI = np.array(axis_i)
        ind.axisD = np.array(axis_d)
        ind.axisSum = np.array(axis_sum)

        ind.motor = np.array(motor)
        ind.debug = np.array(debug)
        ind.heading = np.array(heading)

        # Number of original columns before additional signals are added
        ind_cntr = len(columns)

        return ind, ind_cntr

    def _preprocess_data(self, ind_cntr: int):
        """
        Preprocess loaded data.
        """

        # =============================================================
        # Additional Indices
        # =============================================================
        # Expand indices for additional calculated data columns.
        # =============================================================

        

        self.ind.sinarg = self.ind.debug[0]

        # =============================================================
        # Debug Signal Remapping
        # =============================================================
        
        #self.ind.currentAngle = np.array([
        #    self.ind.debug[1],
         #   self.ind.debug[4],
        #])

        #self.ind.angleTarget = np.array([
        #    self.ind.debug[2],
        #    self.ind.debug[5],
        #])

        #self.ind.angleRate = np.array([
        #    self.ind.debug[3],
        #    self.ind.debug[6],
        #])

        # =============================================================
        # Future Debug Mapping
        # =============================================================

        # Future version would be:
        self.ind.currentAngle = np.array([
             self.ind.debug[4],
             self.ind.debug[6],
        ])
        
        self.ind.angleTarget = np.array([
             self.ind.debug[5],
             self.ind.debug[7],
        ])

        # =============================================================
        # Time Vector
        # =============================================================
        # Convert microseconds to seconds.
        # Time starts at 0 s.
        # =============================================================

        self.time = (
            self.data[:, self.ind.time]
            - self.data[0, self.ind.time]
        ) * 1.0e-6

        # =============================================================
        # Unscale highResolutionGain
        # =============================================================
        # Betaflight can store some signals with increased resolution.
        # If enabled, these signals must be scaled back.
        # =============================================================

        if self.para.get_int("blackbox_high_resolution"):

            blackbox_high_resolution_scale = 10.0

            ind_bb_high_res = np.concatenate([
                self.ind.gyroADC,
                self.ind.gyroUnfilt,
                self.ind.rcCommand,
                self.ind.setpoint[:3],
                self.ind.currentAngle[:2],
                self.ind.angleTarget[:2],
            ])

            self.data[:, ind_bb_high_res] = (
                self.data[:, ind_bb_high_res]
                / blackbox_high_resolution_scale
            )

        # =============================================================
        # Unscale and Remap sinarg
        # =============================================================

        sinarg_scale = 5.0e3

        self.data[:, self.ind.sinarg] = (
            self.data[:, self.ind.sinarg]
            / sinarg_scale
        )

        # =============================================================
        # Unscale and Remap Heading
        # =============================================================

        if len(self.ind.heading) >= 3:
            self.data[:, self.ind.heading[:3]] = (
                self.data[:, self.ind.heading[:3]]
                * 100
            )

        # =============================================================
        # Create Additional Entry for PI Sum
        # =============================================================
        # axisSumPI is created from:
        #     axisP + axisI
        # =============================================================

        # Create an additional entry for the PI sum
        pi_sum = (
            self.data[:, self.ind.axisP]
            + self.data[:, self.ind.axisI]
        )

        self.data = np.column_stack([
            self.data,
            pi_sum,
        ])

        # Now assign the new indices from actual data size
        self.ind.axisSumPI = np.arange(
            self.data.shape[1] - 3,
            self.data.shape[1]
)

        # =============================================================
        # Create Different Sampling Times
        # =============================================================

        Ts = self.para.get_int("looptime") * 1.0e-6

        # Control loop sampling time
        self.Ts_cntr = (
            self.para.get_int("pid_process_denom")
            * Ts
        )

        # Logging loop sampling time
        self.Ts_log = (
            self.para.get_int("frameIntervalPDenom")
            * self.Ts_cntr
        )