# Spectral Analysis of Gyro and Control Signals

To better understand the dynamic behavior of a system, it is often useful to analyze how it responds to inputs at various frequencies. In this implementation, the `estimate_spectra` function is used to perform this frequency-domain analysis [1].


## Overview of the Algorithm

The function processes each signal column independently and performs the following steps:


### Mean Removal

A global mean is removed from every signal. Additionally, each segment undergoes per-segment mean removal, which suppresses low-frequency drift and numerical bias. (DC components are therefore intentionally attenuated.)

---

### Windowing of Each Segment

Before the FFT is computed, each data segment is multiplied by the chosen analysis window (for example, a periodic Hann window). The window reduces unwanted frequency spreading by making the signal smoothly fade in and out at the edges of each segment. Without this tapering, sudden jumps at the segment boundaries would create artificial frequency components that do not exist in the original signal.

$$x_{\mathrm{win}}(n) = x(n) \cdot w(n), \qquad n = 0, \ldots, \text{Nest}-1$$

A periodic Hann window is commonly used because it provides:

- Smooth tapering with minimal discontinuities at the segment boundaries  
- Reduced leakage compared to rectangular windows  
- Good balance of main-lobe width and side-lobe suppression  

After windowing, the segment is ready for amplitude-correct spectral estimation.

---


### Segmentation with Overlap

The input signal is divided into multiple analysis windows (segments) of length $\text{Nest}$. These segments can overlap to improve statistical stability and reduce variance in spectral estimates. A 50% overlap is widely recommended in the literature for Welch’s method [2, pp. 419–434], while overlaps of 75% are also frequently used in practical implementations to further reduce variance. Increasing overlap up to 100% can provide maximum variance reduction, as noted in frequency-domain FRF measurement techniques [1, pp. 62–63, 239].

The number of overlapping samples is defined as $\text{N}_{\text{Overlap}}$. Accordingly, the shift between two consecutive segments is:

$$\text{N}_{\text{Shift}} = \text{Nest} - \text{N}_{\text{Overlap}}$$

Higher overlaps improve smoothness and consistency but increase computational cost. The choice depends on the desired trade-off between variance reduction and efficiency.

---


### Windowing and Amplitude-Correct FFT Scaling

The function uses a custom normalization: 

$$W = \frac{\sum w}{2}$$

The FFT output is then scaled by:

$$\frac{1}{Nest \cdot W}$$

This scaling intentionally pre-doubles all bins so that interior bins already contain the correct single-sided amplitude representation.
However, the **DC** and **Nyquist** bins must not be doubled and are therefore corrected:

$$P_{\mathrm{DC}} = \frac{P_{\mathrm{DC}}}{4}$$
$$P_{\mathrm{Nyq}} = \frac{P_{\mathrm{Nyq}}}{4}$$

This correction ensures:

- Correct amplitude level for sinusoidal bins  
- Correct energy conservation  
- Proper handling of real-valued signals with even FFT length

These characteristics are well documented in frequency-domain signal analysis literature [2].

---

### One-Sided Power Spectrum Construction

From the two-sided FFT power spectrum, only the positive half is used:

$$P_{\text{1-sided}}(k) = |U(k)|^{2}$$

The power is averaged across all segments:

$$P_{\text{avg}}(f) = \frac{1}{N_{\text{segments}}}\sum P_{\text{seg}}(f)$$

The output is:

- **$P_{avg}$**: one-sided power spectrum  
- **$freq$**: matching frequency vector  

To obtain single-sided amplitude spectra:

$$A(f) = \sqrt{P_{\text{avg}}(f)}$$

This form is used throughout the analysis for gyro data, filtered gyro signals and control-loop sums [1].


## References

[1] Pintelon, R., & Schoukens, J. *System Identification: A Frequency Domain Approach*  
    (2nd ed.). Wiley-IEEE Press, 2012, pp. 45–60, 118–121, 62–63, 239.

[2] Hayes, M. H. *Statistical Digital Signal Processing and Modeling*.  
    Wiley, 1996, pp. 455–457.