# Step Response from Frequency Response Data

The function `calculate_step_response_from_frd` approximates the **time-domain step response** of the newly calculated transfer function in [Closed-Loop Analysis](./CalculateClosedLoop.md). The **step response** illustrates how the system reacts to a sudden change during a unit step and therefore provides a direct view of the **transient behavior** of the control loop. From this response, key performance indicators such as **rise time**, **overshoot**, **settling time** and **steady-state error** can be observed. A smooth and fast rise indicates a well-tuned controller, whereas overshoot or oscillations reveal underdamped dynamics or insufficient phase margin [1].


## Methods

Through the calculations in the previous steps, the complete **closed-loop transfer function** of the system has already been obtained. Based on this transfer function, it is now possible to determine the **step response**, which describes how the system reacts to a sudden change in the input signal. 

To compute this response, the transfer function is transformed from the **frequency domain** back into the **time domain** using the **inverse Fourier transform**. This transformation yields a discrete-time representation corresponding to the system’s impulse response $g[k]$, which characterizes how the system reacts to an instantaneous excitation.  

Since the step input represents a discrete unit step, the step response can be obtained by **accumulating the impulse response samples in time**. Accordingly, the step response $y[k]$ is calculated as the cumulative sum of the impulse response:

$$y[k] = \sum_{i=0}^{k} g[i]$$

In this form, the step response reflects the combined effects of all filters, delays and feedback mechanisms within the control loop. A well-damped and smooth curve indicates a stable and responsive system, whereas oscillations or overshoot reveal underdamped or poorly tuned controller dynamics [2].

---

### Normalization of the Step Response

To simplify the analysis of the transient behavior, the computed step response is **normalized using its steady-state value**. The steady-state value is estimated as the **mean value in a time window after the transient has settled**, which provides a robust reference.

The normalization is performed by dividing the entire step response by this steady-state value:

$$y_\text{norm}[k] = \frac{y[k]}{\bar{y}_\text{ss}}$$

After normalization, the step response **approaches 1 in steady state**, representing a unit step response. This removes the influence of the absolute system gain and makes it easier to compare **rise time**, **overshoot** and **settling behavior** between different controller settings.

The focus is therefore on the **dynamic behavior of the closed-loop system**, rather than on absolute amplitudes.


## Compliance

The **compliance** is calculated within the function `calculate_closed_loop` and describes how much the controlled system **deflects under an applied external force or disturbance**.  
In mechanical terms, compliance is the **inverse of stiffness** and is defined as:

$$SP = \frac{\text{Displacement}}{\text{Force}} = \frac{1}{k}$$

In control theory, compliance represents the **transfer function from an external disturbance force to the resulting system displacement**. It expresses how easily the system yields to external influences and therefore serves as a measure of **disturbance sensitivity**.

A **low compliance** indicates a **stiff and disturbance-resistant system**, effectively rejecting external forces such as wind or vibration. However, such systems may be more prone to oscillations if the controller bandwidth is too high. A **high compliance**, on the other hand, corresponds to a **soft or flexible system** that responds more smoothly but is less capable of rejecting disturbances. In the context of closed-loop control, analyzing compliance helps to balance **disturbance rejection**, **stability** and **responsiveness** key objectives when tuning and validating feedback systems. [3]


## References

[1] H. Lutz and W. Wendt, Taschenbuch der Regelungstechnik, 6th ed., Springer Vieweg, Berlin, Heidelberg, pp. 60.

[2]  H. Lutz and W. Wendt, Taschenbuch der Regelungstechnik, 6th ed., Springer Vieweg, Berlin, Heidelberg, pp. 98.

[3] K. J. Åström and R. M. Murray, Feedback Systems: An Introduction for Scientists and Engineers, Princeton University Press, 2008, ch. 12, sec. 12.1, p. 403 (PDF). Available: https://fbsbook.org
