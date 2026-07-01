# Closed-Loop Analysis

The goal of the function `calculate_closed_loop` is to calculate the different parts of the complete closed-loop control system, including its transfer functions, sensitivity functions, compliance and controller effort. This allows us to analyze the different parts of the control system and understand how they interact to shape the overall system behavior.  
This structure separates the control actions into an **inner loop** and an **outer loop**.


## Transfer Functions

The transfer functions describe how a signal is transformed as it passes through the different components of the control system.
To calculate the transfer function of the system, we have to split the control loop into an **inner** and an **outer** loop.
First we have to calculate the transfer function of the inner loop (path from `u` to `y`). [1]

$$L_{in}(\omega) = P(\omega) \cdot G_{f}(\omega)$$ $$\qquad G_{yu}(\omega) = \frac{L_{in}(\omega)}{1+L_{in}(\omega) \cdot G_{fD}(\omega) \cdot C_{D}(\omega)}$$

With this information we are able to calculate the whole closed loop system.

$$L_{out}(\omega) = G_{yu}(\omega) \cdot C_{PI}(\omega)$$ $$\qquad G_{yw}(\omega) = \frac{L_{out}(\omega)}{1+L_{out}(\omega)}$$

<p align="center">
  <img src="./Images/Regelstrecke.png"
     alt="ZHAW Logo"
     width="800"
     style="float:center; margin-left:20px; margin-right:20px;
     margin-top:20px;">
</p>


## Sensitivity

The sensitivity describes how the closed-loop system reacts to disturbances and noise. To achieve good disturbance rejection and accurate reference tracking, the sensitivity should be low at low frequencies, while higher sensitivity at high frequencies helps to avoid noise amplification and maintain robustness. [2]

To calculate the sensitivity functions of the inner and outer loop, we use the following formulas:

$$S_{in}(\omega) = \frac{1}{1+L_{in}(\omega)}$$
$$S_{out}(\omega) = \frac{1}{1+L_{out}(\omega)}$$


## Controller Effort

The controller effort indicates how much effort the controller has to exert to maintain the desired system performance. It is important to monitor this value to ensure that the controller does not exceed its physical limits, which could lead to instability or damage to the system [3].

The controller effort can be calculated using the following formula:

$$SC(\omega) = C_{PI}(\omega) \cdot S_{out}(\omega)$$


## Compliance

The compliance describes how the system responds to external disturbances or changes in the reference input. A high compliance indicates that the system is able to adapt quickly to changes, while a low compliance indicates that the system is more rigid and less responsive [3].

The compliance can be calculated using the following formula:

$$SP(\omega) = G_{yu}(\omega) \cdot S_{out}(\omega)$$


## References

[1] H. Lutz and W. Wendt, Taschenbuch der Regelungstechnik, 6th ed., Springer Vieweg, Berlin, Heidelberg, pp. 183–186.

[2] K. Stadler, Control Theory I (Regelungstechnik I), Zurich University of Applied Sciences (ZHAW), Rev. 7.13, 2025, Sec. 7.3.1 “Sensitivity & complementary sensitivity limits”, p. 130.

[3] K. J. Åström and R. M. Murray, Feedback Systems: An Introduction for Scientists and Engineers, Princeton University Press, 2008, ch. 12, sec. 12.1, p. 403 (PDF). Available: https://fbsbook.org
