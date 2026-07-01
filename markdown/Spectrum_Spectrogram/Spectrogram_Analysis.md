# Spectrogram Estimation

The function `estimate_spectrogram` extends the spectral estimation described in [Spectra Analysis](./Spectra_Analysis.md) by computing a **single-sided, amplitude-correct spectrogram** over an additional coordinate $y$. This allows the frequency content of a signal to be analyzed not only over time but also along another dimension, such as height, position, angle, or system state.

Instead of forming a single averaged spectrum, the algorithm groups all FFT segments into $N_{\text{res}}$ bins along the thrust axis and computes one spectrum per bin.  
The amplitude calibration, FFT scaling and DC/Nyquist correction follow the same method as in `estimate_spectra`.


## Function Overview

### 1. Binning Along the Secondary Coordinate

The input vector $y$ is divided into $N_{\text{res}}$ equally spaced bins:

$$y_{\text{axis}} = \left[\, y_{\min},\, y_{\min} + \Delta y, \ldots, y_{\max} - \Delta y \,\right]$$
$$\Delta y = \frac{y_{\max} - y_{\min}}{N_{\text{res}}}$$

Each FFT segment is assigned to one or more of these bins depending on its corresponding $y$ values.

---

### 2. Segmentation, Windowing and FFT

Segmentation, windowing with a tapered analysis window (e.g., Hann) and amplitude-correct FFT scaling follow the same steps described in the [Spectra Analysis](./Spectra_Analysis.md) chapter. [2, pp. 455–457]

Each segment produces a one-sided power vector:

$$P_{\text{seg}}(f) = |U(f)|^2$$

with DC and Nyquist bins corrected as:

$$P_{\mathrm{DC}} = \frac{P_{\mathrm{DC}}}{4}$$

$$P_{\mathrm{Nyq}} = \frac{P_{\mathrm{Nyq}}}{4}$$

---

### 3. Accumulation Into $y$-Bins

For each segment, all samples of $y$ belonging to that segment are mapped to their corresponding $y$ bins.  
The resulting segment spectrum is added to those bins, weighted by the number of matching samples:

$$P_{\text{avg}}(y_i, f) = \frac{1}{N_i} \sum_{k \in \mathcal{S}_i} P_{\text{seg},k}(f)$$

where $N_i$ is the number of contributing segment samples for bin $i$.

This creates a 2D matrix:

$$P_{\text{avg}} \in \mathbb{R}^{N_{\text{res}} \times N_{\text{freq}}}$$

representing the power spectrum as a function of both frequency and $y$. [1, pp. 45–48][2, pp. 455–457]

---

### 4. 2D Smoothing of the Spectrum

A small weighted 3×3 convolution kernel is applied to reduce noise and produce a visually smooth spectrogram.  
The kernel preserves energy by normalizing the sum of all weights.

$$K =
\begin{bmatrix}
1 & 3 & 1 \\
3 & 5 & 3 \\
1 & 3 & 1
\end{bmatrix}$$

$$K_\text{norm} = \frac{K}{\sum K}$$

The smoothed spectrogram is obtained as:

$$P_{\mathrm{smooth}} = \frac{P_{\mathrm{avg}} * K_\text{norm}}{\mathbf{1} * K_\text{norm}}$$


## Outputs

- **$P_{avg}$** — the averaged single-sided power spectrogram  
- **$freq$** — corresponding frequency axis ($0 … \frac{f_s}{2}$)  
- **$y_{axis}$** — center coordinates of the $y$ bins  

Amplitude spectrograms can be obtained via:

$$A(y,f) = \sqrt{P_{\text{avg}}(y,f)}$$


## References

[1] R. Pintelon, J. Schoukens, *System Identification: A Frequency Domain Approach*,  
2nd ed., IEEE Press, 2012, pp. 45–60, 62–63, 239.

[2] M. H. Hayes, *Statistical Digital Signal Processing and Modeling*,  
John Wiley & Sons, 1996, pp. 51–52, 455–457.
