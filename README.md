# bf_controller_tuning

WIP

A huge ass script to analyse **bbl** flight logs and tune linear filters and pids offline. This libary is intendet to be used with `Chirp signal generator as flight mode #13105 (Merged)`.

### Required Matlab Toolboxes

- MATLAB
- Control System Toolbox
- Signal Processing Toolbox

##

# Chirp Signal Generator as flight mode

## Concept
* An automatic chirp signal generator is used to excite the quadcopter during flight. The chirp frequency increases exponentially over a defined period of time, starting from a specified starting frequency and ending at a specified ending frequency.
* The chirp signal generator is the module that generates the chirp signal.
* The chirp signal is added to the ``currentPidSetpoint`` in the ``pid.c`` file. So it is still possible to steer the quadcopter while performing the measurement. The system's behavior is tested when flying in both ``ACRO`` and ``ANGLE`` modes.
* Since the typical rate-controlled closed-loop system has differentiating behavior from the rate pidSetpoint to the pidSum in the low-frequency range up to approximately 30 Hz, the chirp signal is shaped using a Lag Filter before applied to the loop.
* The chirp signal generator is implemented as a feature. To enable it, you need to include ``chirp`` in the custom defines or use the ``-DUSE_CHIRP`` flag if you are building locally.
* It is recommended to assign the ``CHIRP`` mode to a non-momentary switch. When the ``CHIRP`` mode is enabled for the first time using the switch, the chirp signal is applied to the roll axis. After disabling and enabling the switch again, the chirp signal is applied to the pitch axis. Next, it is applied to the yaw axis, and then it starts again with the roll axis. So it is possible to cycle through all 3 axis severel times if needed. The ``CHIRP`` mode can be disabled and re-enabled at any time.
* When the ``CHIRP`` mode is enabled, ``CHIR`` is displayed as the flight mode in the goggles. When the chirp signal is finished, a blinking warning saying ``CHIRP IS FINISHED`` is shown and ``CHIR`` disapears.
* The chirp mode can be tested on the bench without the propellers, using decreased PID gains (P=10, I=0, D=0), `chirp_amplitude_roll = chirp_amplitude_pitch = chirp_amplitude_yaw = 10` while in ``ACRO`` mode.

## CLI Parameters
Name | Default Value | Short Explanation
-- | - | - |
**chirp_lag_freq_hz** | 3 Hz | leadlag1Filter cutoff/pole to shape the excitation signal
**chirp_lead_freq_hz** | 30 Hz | leadlag1Filter cutoff/zero
**chirp_amplitude_roll** | 230 deg/sec | amplitude roll in degree/second
**chirp_amplitude_pitch** | 230 deg/sec | amplitude pitch in degree/second
**chirp_amplitude_yaw** | 180 deg/sec | amplitude yaw in degree/second
**chirp_frequency_start_deci_hz** | 0.2 Hz / 2 deciHz | start frequency in units of 0.1 hz
**chirp_frequency_end_deci_hz** | 600 Hz / 6000 deciHz | end frequency in units of 0.1 hz
**chirp_time_seconds** | 20 sec | excitation time

## Assumptions for pichim's offline Tuning
- Dynamic Notch filters are set and tuned (time-variant)
- RPM filters are set and tuned (time-variant)
- Thrust Linear is set and tuned (nonlinear)
- Iterm Relax is set and tuned (nonlinear)
- FF is off (nonlinear)
- Dynamic Damping is off -> both D max values are the same as the D values or equal to 0 (nonlinear)
- Debug is set to CHIRP (``set debug_mode = CHIRP``)
- Use highresolution for logging (``set blackbox_high_resolution = ON``)
- It is important to let the chirp signal run for the hole time ``chirp_time_seconds``

## Tuning the CHIRP Generator
* It is important to ensure that the pid output or the motors do not become saturated during the experiment. The amplitude of the chirp signal and the lag filter can be adjusted accordingly to prevent motor saturation.

## Usual testing procedure within one BB-log file / flight
1. Perform two to three throttle sweeps while in ``ACRO`` mode.
2. Complete at least one full sequence of chirp signal excitation, covering roll, pitch, and yaw axes. It is preferable to cycle through all axes twice. Whether you choose ``ACRO`` or ``ANGLE`` mode does not matter. Fly in an open space and try to maintain altitude. Be prepared to adjust the throttle as the chirp generator runs. Aim for a smooth and steady flight during the chirp excitation. Ideally, the quadcopter should maintain a steady position and orientation (appart from the axes thats excited).
3. Conduct some maneuvers to test propwash handling, including 180-degree and 360-degree flips. Enjoy yourself and have fun!

## What I need to do the offline Tuning
* *.BBL log

## Repositories for People that have Matlab available
- https://github.com/pichim/bf_controller_tuning

## Link to an example flight
- https://www.youtube.com/watch?v=bU63eY66QX0

## Links to Chirp Signal Theory
- https://en.wikipedia.org/wiki/Chirp
- https://ch.mathworks.com/help/signal/ref/chirp.html