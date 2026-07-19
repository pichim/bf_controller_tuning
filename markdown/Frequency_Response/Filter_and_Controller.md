# Filter and Controller Structure

The functions `get_filter` and `calculate_transfer_functions` together form the basis for constructing the controller and filter structure in the closed control loop.  
Both follow the filter and controller architecture used in **Betaflight** [1].


## Function `get_filter`

This function generates a **discrete filter** section of a specified type and returns it as both a state-space model and a transfer function.  
The filters are implemented according to the Betaflight filter architecture and frequency characteristics [1].

According to **Betaflight**, the user can choose which of the following filter types to apply depending on the axis and signal path [1]:

- **PT1, PT2, PT3:** Low-pass filters with a matched cutoff frequency (the –3 dB point is identical for all PT variants) [1]. They attenuate high-frequency components while maintaining a smooth phase response. Higher-order variants (PT2, PT3) provide a steeper attenuation beyond the cutoff frequency, consistent with their system order [2].

- **Biquad:** Second-order low-pass filter with a **fixed** $Q = \frac{1}{\sqrt{2}}$ [1]. The filter exhibits a magnitude attenuation of **–40 dB per decade** at higher frequencies, which is characteristic of a second-order system and shows **no pronounced overshoot in the time domain** [2].

- **Notch:** Band-stop (notch) filter with a quality factor $Q$ computed by `get_notch_Q` [1]. Used to suppress narrow-band resonances such as structural vibrations or motor-induced noise.

- **Lead / Lag:** Phase compensation or lead–lag network [1]. These filters modify the phase response (lead or lag) to improve closed-loop stability or to compensate for delays in the control loop.

All filters are discretized using the sample time $T_s$ [1]. The function `get_filter` is called internally by `calculate_transfer_functions` to build the gyro, D-term and P-term filter paths [1].


## Function `calculate_transfer_functions`

This function constructs the **filter chains and controllers** for a selected axis according to the Betaflight control structure [1].  
It creates the following filter paths:

1. **Gyro path $G_{fg}$:** Low-pass filters, dynamic low-pass filters, notch filters and optional phase compensation [1].

2. **D-term path $G_{fd}$:** Filter chain for the derivative path, cascaded with the derivative controller $C_D$ [1].

From the axis parameters, the discrete **PI** and **D** controllers are constructed as standard discrete-time control elements [2]:

$$C_{PI}(z) = K_p + K_i \cdot T_s \frac{z}{z-1}$$

$$C_D(z) = \frac{K_d}{T_s} \frac{1 - z^{-1}}{z^{-1}}$$

The function returns the state-space models $C_{PI}$, $C_D$ and $G_f$, as well as the effective PID gain vector.


## References

[1] Betaflight Development Team, *Betaflight Flight Controller Firmware*,  
GitHub Repository, https://github.com/betaflight/betaflight (accessed: 2025-12-2025).

[2] H. Lutz and W. Wendt, Taschenbuch der Regelungstechnik, 6th ed., Springer Vieweg, Berlin, Heidelberg, pp. 183–186.

