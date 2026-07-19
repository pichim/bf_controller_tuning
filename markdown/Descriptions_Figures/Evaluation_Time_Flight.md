# Evaluation Time Flight

This plot shows the timing accuracy of the flight controller throughout the entire flight. Each value represents the time difference between two logged samples, measured in microseconds. This allows you to see how stable and consistent the controller’s loop time was.

A stable loop time is essential because the PID controller relies on consistent sampling intervals for correct operation. Variations in loop time can affect flight performance and sudden spikes may indicate processor overload, logging delays, or sensor issues. 

## Loop Time Values

### Mean (Average Loop Time)

The expected loop time depends on the configured PID loop frequency:

$T_s = \frac{1}{f}$

| Loop Frequency | Expected Mean |
|----------------|---------------|
| **1 kHz**      | ~ **1000 µs** |
| **2 kHz**      | ~ **500 µs**  |
| **4 kHz**      | ~ **250 µs**  |
| **8 kHz**      | ~ **125 µs**  |

These values represent the target loop time (`desiredPeriodUs`) configured in the Betaflight scheduler.

---

### Median
- Should be close to the mean value
- Large differences between the mean and median may indicate timing outliers or sudden spikes.

---

<p align="center">
  <img src="./Images/evaluation_time_flight.jpg"
     alt="Flight controller loop time over entire flight"
     width="1000"
     style="float:center; margin-left:10px; margin-right:10px;">
</p>


## Sources

- Betaflight source code
  - [src/main/scheduler/scheduler.c](https://github.com/betaflight/betaflight/blob/master/src/main/scheduler/scheduler.c)
  - [src/main/scheduler/scheduler.h](https://github.com/betaflight/betaflight/blob/master/src/main/scheduler/scheduler.h)
