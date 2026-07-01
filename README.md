# Betaflight Controller Tuning

## Overview
This repository provides an **offline controller tuning framework** for Betaflight.
It enables analysis of **Blackbox (BBL) flight logs** with automatic chirp excitation,
to tune **linear filters** and **PID controllers** offline.

The default workflow covers both the **rate loop (Gyro/ACRO)** and the **attitude loop (Angle)**.
AltHold and PosHold are currently proof-of-concept extensions (see note below).

The workflow:

1. Enable and configure the **chirp signal generator** in Betaflight ([see implementation below](#implementation-of-the-chirp-signal-generator-in-betaflight)).
2. Perform flights with chirp excitation and log data (`.bbl` files).
3. Convert `.bbl` files to `.csv` using the [Betaflight Blackbox Explorer]
4. Use one of the entry scripts [`main.m`](./main.m), [`main.py`](./main.py), or [`main.ipynb`](./main.ipynb) together with the functions in [`lib/`](./lib), [`class`](./class) and [`py_lib/`](./py_lib) to:
   - Extract log data
   - Compute frequency responses and spectra
   - Display spectra and spectrograms
   - Estimate closed-loop behavior for the rate loop (Gyro/ACRO)
   - Estimate closed-loop behavior for the attitude loop (Angle), including the outer P-controller
   - Extract plant (quad) dynamics seen from the pids
   - Extract measured controller dynamics and compare to the theoretical (split in PI and D 2-DOF controller for the rate loop)
   - Enables offline PID and filter tuning
   - Display step responses of setpoint tracking and input disturbance rejection (compare actual tune to a new user specified tune)
   - Display Bode plots of closed-loop system, e.g. Tracking, Sensitivity, Controller Effort, Compliance (compare actual tune to a new user specified tune)

This repository is intended for use with [Betaflight PR #13105](https://github.com/betaflight/betaflight/pull/13105), which adds the chirp generator mode.

**Important branch note:**
The default/main workflow in this README covers **Gyro (rate loop)** and **Angle (attitude loop)** tuning. **AltHold** and **PosHold** tuning/evaluation are proof-of-concept and available only in the `proof-of-concept` branch of this repository.

The corresponding firmware is a fork of upstream Betaflight, based on the master branch from February 2026 (commit `04e37a122`), with AltHold and PosHold chirp functions added in the separate `althold_chirp` and `poshold_chirp` branches of [InsaneBroccoli/betaflight](https://github.com/InsaneBroccoli/betaflight). These functions are **not part of official Betaflight** and are not merged into any upstream release or the Betaflight Configurator. Because of this, the firmware for AltHold/PosHold testing cannot be flashed the usual way (Configurator + custom `CHIRP` define on an official release). Instead, you need to build the firmware yourself from the respective branch and flash it manually.

The configurator and the blackbox explorer can be found here:

- [Betaflight Configurator](https://master.app.betaflight.com/)
- [Betaflight Blackbox Explorer](https://master.blackbox.betaflight.com/)

## Requirements
- **MATLAB** (for `main.m`, last tested with version R2024a)
- **Control System Toolbox**
- **Signal Processing Toolbox**
- **Python** (for `main.py` / `main.ipynb`), tested with Python 3.12

The Python environment is managed via conda and defined in [`envs/tuning.yml`](./envs/tuning.yml):

```yaml
name: tuning
channels:
  - conda-forge
dependencies:
  - python=3.12
  - pandas=2.1.4
  - numpy=1.26.2
  - matplotlib=3.8.2
  - scipy=1.11.4
  - control=0.10.0
```

Create and activate the environment with:

```bash
conda env create -f envs/tuning.yml
conda activate tuning
```


## Implementation of the CHIRP Signal Generator in betaflight
The chirp signal generator in Betaflight provides an automated excitation input for analyzing the quadcopter's dynamic response. It produces a sweep signal whose frequency rises exponentially from a defined starting point to a chosen final value over a preset duration.

This generator outputs a signal that is added directly to currentPidSetpoint inside pid.c. Because of this, the pilot is still in full control over the aircraft during the measurement, and the test can be performed in both `ACRO` and `ANGLE` flight modes.

Because a typical rate-controlled closed-loop system exhibits differentiating behavior from `pidSetpoint` to `pidSum` at low frequencies (up to around 30 Hz), the chirp signal is shaped by a Lag Filter before being injected into the loop.

In order to bring the `CHIRP` signal generator onto your drone, you must include it as a custom define while flashing the Betaflight 2025.12.xx Firmware. You can just type CHIRP into the Field under User Definitions (see picture below). After flashing the Firmware, the `CHIRP` mode appears in the modes tab.

It is recommended to assign the `CHIRP` mode to a non-momentary switch. During the first activation, the chirp is applied to the roll axis. After toggling the switch off and on, the signal is applied to the pitch axis and then to the yaw axis. After one full cycle it resets to the roll axis again. This allows repeated analysis of all axes as needed. The mode can be turned off and reactivated at any time.

When `CHIRP` mode is active, the goggles show CHIR as flight-mode label. Once the chirp sequence is complete (after 20 sec), a blinking message CHIRP IS FINISHED is displayed and the flight-mode label changes back to current flight mode.

If you want to test the mode before flight, it is also possible to do this safely on the bench. To prevent damages to your components, lower the PID gains (e.g., P = 10, I = 0, D = 0), and set the values for chirp_amplitude_roll, chirp_amplitude_pitch, and chirp_amplitude_yaw to 10 while the quadcopter is in `ACRO` mode.

## CLI Paramters

### Gyro and Angle modes

| Name                            | Default Value        | Explanation                                               |
| ------------------------------- | -------------------- | --------------------------------------------------------- |
| `chirp_lag_freq_hz`             | 3 Hz                 | leadlag1Filter cutoff/pole to shape the excitation signal |
| `chirp_lead_freq_hz`            | 30 Hz                | leadlag1Filter cutoff/zero                                |
| `chirp_amplitude_roll`          | 230 deg/sec          | amplitude roll in degree/second                           |
| `chirp_amplitude_pitch`         | 230 deg/sec          | amplitude pitch in degree/second                          |
| `chirp_amplitude_yaw`           | 180 deg/sec          | amplitude yaw in degree/second                            |
| `chirp_frequency_start_deci_hz` | 0.2 Hz / 2 deciHz    | start frequency in units of 0.1 hz                        |
| `chirp_frequency_end_deci_hz`   | 600 Hz / 6000 deciHz | end frequency in units of 0.1 hz                          |
| `chirp_time_seconds`            | 20 sec               | excitation time                                           |

These parameters apply to both flight modes; the closed loop under test depends on whether `ACRO` (rate loop) or `ANGLE` (attitude loop) is active during the chirp.

### Altitude Hold mode (proof-of-concept branch only)
| Name                       | Default Value         | Explanation                                               |
| -------------------------- | --------------------- | --------------------------------------------------------- |
| `altChirpAmpl`             | 150 cm                | amplitude in cm                                           |
| `altChirpStartFreqHzCenti` | 0.05 Hz / 5 centiHz   | start frequency in units of 0.01 hz                       |
| `altChirpEndFreqHzDeci`    | 3 Hz / 30 deciHz      | end frequency in units of 0.1 hz                          |
| `altChirpSweepTimeSec`     | 75 sec                | excitation time                                           |

### Position Hold mode (proof-of-concept branch only)
| Name                       | Default Value         | Explanation                                               |
| -------------------------- | --------------------- | --------------------------------------------------------- |
| `posChirpAmpl`             | 100 cm                | amplitude in cm                                           |
| `posChirpStartFreqHzCenti` | 0.05 Hz / 5 centiHz   | start frequency in units of 0.01 hz                       |
| `posChirpEndFreqHzCenti`   | 3 Hz / 300 centiHz    | end frequency in units of 0.01 hz                         |
| `posChirpSweepTimeSec`     | 75 sec                | excitation time                                           |
| `posChirpYawP`             | 2                     | P value of P controller for automatic north alignment     |
| `posChirpMaxYawRate`       | 90 deg/sec            | max yaw rate for automatic north alignment                |
| `posChirpAlignTolerance`   | 3 deg                 | heading tolerance                                         |
| `posChirpLagFreqHzDeci`    | 1.7 Hz / 17 deciHz    | leadlag Filter cutoff/pole to shape the excitation signal |
| `posChirpLeadFreqHzDeci`   | 3 Hz / 30 deciHz      | leadlag Filter cutoff/zero                                |
| `posChirpHeading`          | 0 deg (= North)       | heading in which the drone faces during chirp excitation  |

## Assumptions for Offline Tuning
- Dynamic Notch filters are tuned (time-variant)
- RPM filters are tuned (time-variant)
- Thrust Linear is tuned (nonlinear)
- Iterm Relax is tuned (nonlinear)
- Feedforward (FF) is disabled (nonlinear)
- Dynamic Damping is disabled (Dmax = D or 0) (nonlinear)
- Debug mode is set to CHIRP (set debug_mode = CHIRP)
- Blackbox high-resolution logging is enabled (set blackbox_high_resolution = ON)
- Let chirp run for the full chirp_time_seconds

## Tuning Recommendations
- Ensure motors/outputs do not saturate during chirp.
- Adjust amplitude and lag filter if saturation occurs.

## Recommended procedure within one log file per flight
1. Perform two to three throttle sweeps while in `ACRO` mode.
2. Complete at least one full sequence of chirp signal excitation, covering roll, pitch, and yaw axes. It is preferable to cycle through all axes twice. Whether you choose ACRO or ANGLE mode does not matter. Fly in an open space and try to maintain altitude. Be prepared to adjust the throttle as the chirp generator runs. Aim for a smooth and steady flight during the chirp excitation. Ideally, the quadcopter should maintain a steady position and orientation (appart from the axes thats excited).
3. Conduct some maneuvers to test propwash handling, including 180-degree and 360-degree flips. Enjoy yourself and have fun!

## Data for evaluation
To evaluate your flight data, the log file from the Blackbox is required. This has to be converted to a .csv file. You can do this with the [Betaflight Blackbox Explorer](https://blackbox.betaflight.com/).

# Tuning via MATLAB / Python

You can run the tuning workflow with:
- `main.m` (MATLAB)
- `main.py` (Python)
- `main.ipynb` (Jupyter Notebook)

1. Open one of the entry files (`main.m`, `main.py`, or `main.ipynb`).
2. Define path and filename of your log file.
3. Select axis/mode-related options.
4. Run evaluation and inspect plots.
5. Adjust parameters and compare baseline vs new tune.

For the MATLAB workflow (`main.m`), the most important options are:
- `ind_ax`: selects the axis (Roll = 1, Pitch = 2, Yaw = 3)
- `angle_tuning_required`: if `true`, Angle-loop tuning is additionally computed after Gyro-loop tuning (cascaded, using the Gyro-loop result as inner loop)
- `do_compensate_iterm`: enables/disables I-Term Relax compensation (rate loop)
- `default_parameters_gyro`: if `true`, baseline and candidate rate-loop tune start with the same PID/filter parameters (recommended for first comparison)
- `default_parameters_angle`: if `true`, baseline and candidate angle-loop tune start with the same P gain
- Filter settings (`para_new.*`): select filter types and adjust frequencies for the rate loop
- PID settings (`P_new`, `I_new`, `D_new`, per axis): set or scale Roll/Pitch/Yaw PID values for the rate loop, compared against the logged baseline
- `P_Angle`: sets the P gain for the attitude (Angle) loop controller

Angle-loop tuning only runs if all of the following hold:
- `angle_tuning_required` is `true`
- the loaded log actually contains Angle-mode chirp excitation, checked automatically via the flight-mode flags (`angle_tuning_possible`)
- the selected axis is Roll or Pitch (`ind_ax ~= yaw`), since Betaflight has no Angle mode for yaw

Unlike the rate loop, which is tuned as a PI and D 2-DOF controller, the Angle loop controller is a single P gain combined with a fixed PT3 low-pass filter (50 Hz), reflecting Betaflight's `levelPID` structure.

The Python entry points (`main.py`, `main.ipynb`) mirror these options through `py_lib/gyro_ctrl_tuning.py` and `py_lib/angle_ctrl_tuning.py`.

After running, review the generated figures (see [Descriptions Figures](./markdown/Descriptions_Figures/)) and iterate with updated parameters.
For faster iteration after the first full run, execute only the tuning/plot sections instead of the complete script.

**Proof of Concept extension (AltHold/PosHold):**
For **AltHold** and **PosHold** analysis, use the scripts in the `proof-of-concept` branch.
These modes are **not part of the default/main branch workflow** yet.

## Example Flight
- [YouTube Example](https://www.youtube.com/watch?v=bU63eY66QX0)

## Related Theory
- [Chirp](https://github.com/InsaneBroccoli/bf_controller_tuning/tree/PA_final/markdown/Sinarg)
- [Data](https://github.com/InsaneBroccoli/bf_controller_tuning/tree/PA_final/markdown/Data)
- [Frequency Response](https://github.com/InsaneBroccoli/bf_controller_tuning/tree/PA_final/markdown/Frequency_Response)
- [Spectrum and Spectrogram](https://github.com/InsaneBroccoli/bf_controller_tuning/tree/PA_final/markdown/Spectrum_Spectogram)

## Repository Structure
```tree
bf_controller_tuning/
│
├── class/
│     ├─ angle_ctrl_tuning.m
│     ├─ flight_analyzer.m
│     ├─ flight_data.m
│     ├─ gyro_ctrl_tuning.m
│     └─ plot_utils.m
├── envs/
|     └─ tuning.yml
├── lib/
│     ├─ apply_rotfiltfilt.m
│     ├─ bode_plot_options.m
│     ├─ *calculate_althold_controller.m
│     ├─ calculate_closed_loop.m
│     ├─ calculate_closed_loop_angle.m
│     ├─ calculate_controllers.m
│     ├─ *calculate_poshold_controller.m
│     ├─ calculate_step_response_from_frd.m
│     ├─ calculate_transfer_functions.m
│     ├─ downsample_frd.m
│     ├─ estimate_frequency_response.m
│     ├─ estimate_spectra.m
│     ├─ estimate_spectrogram.m
│     ├─ expand_multiple_figure_nr.m
│     ├─ extract_header_information.m
│     ├─ get_chirp_signals.m
│     ├─ get_fcut_from_D_and_fcenter.m
│     ├─ get_fcut_from_exp.m
│     ├─ get_filter.m
│     ├─ get_ind_eval.m
│     ├─ get_my_colors.m
│     ├─ get_notch_Q.m
│     ├─ get_pid_scale.m
│     └─ get_switch_case_text_from_para.m
├── logs/
│     └─ example_logs/
├── markdown/
│     ├─ Control Loops/
│     ├─ Data/
│     ├─ Descriptions_Figures/
│     ├─ Frequency_Response/
│     ├─ Sinarg/
│     └─ Spectrum_Spectrogram/
├── py_lib/
│     ├─ __init__.py
│     ├─ angle_ctrl_tuning.py
│     ├─ flight_analyzer.py
│     ├─ flight_data.py
│     ├─ gyro_ctrl_tuning.py
│     ├─ pidtuninglib.py
│     └─ plot_utils.py
├── *altitude_hold_tuning.m
├── main.ipynb
├── main.m
├── main.py
└── *position_hold_tuning.m
```

\*  proof-of-concept branch only