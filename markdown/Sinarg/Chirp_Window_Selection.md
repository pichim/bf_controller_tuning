# Chirp Window Selection

For a precise and reliable determination of the transfer function, the use of a **chirp signal** is essential. (For more information, see [Chirp Signal](./Chirp.md)). However, to ensure that only the **actively excited portions** of the signal are used in the evaluation, it is necessary to determine when the chirp excitation was active.  

In Betaflight, the output `debug[0]` was configured to output the **phase of the chirp** $\text{arg}(t)$ during the active chirp phase. This signal serves as a reference to identify and mark the **time intervals of active excitation** for further analysis.


## Identification of Active Excitation Intervals (`get_ind_eval`)

After detecting the `sinarg` signal in the dataset, the next step is to determine which time segments should be included in the analysis. For this purpose, the function `get_ind_eval` is used. 

This function creates a **logical mask (`ind_eval`)** that specifies the time points at which the chirp signal was active and the system was sufficiently excited. It evaluates two main criteria:

1. **Phase window:**  
   The signal `sinarg` (representing the phase of the chirp $\text{arg}(t)$) is used to identify the time intervals in which the chirp excitation was active. Time points with `sinarg > 0` are marked as **active sections** and included in the analysis.

2. **Signal dynamics:**  
   Within these intervals, the **variance** of the measured response signal (for example the gyro signal) is computed. Higher variance indicates that the system is responding dynamically to the input excitation. Only windows whose variance exceeds a defined threshold are considered valid excitation phases.

The function then returns the **index mask** corresponding to these active segments, which is later used to remove all non-active data points from the main analysis. [1]


## Purpose of This Function

The purpose of this preprocessing step is to limit the analysis to those time intervals in which the system was **actively excited by the chirp signal** and exhibited a measurable dynamic response. Without this selection, time periods without excitation would also be included in the estimation, leading to distorted or noisy transfer function results. 

By filtering the data in this way, the method improves the **signal-to-noise ratio (SNR)** and ensures a **stable, reproducible and accurate** identification of the system dynamics [2].


## References

[1] K. J. Åström and R. M. Murray, Feedback Systems: An Introduction for Scientists
    and Engineers, Princeton University Press, 2008, ch. 10, pp. 364–369.

[2] R. Pintelon and J. Schoukens, System Identification: A Frequency Domain
      Approach, 2nd ed., IEEE Press, 2012, pp. 49-53.