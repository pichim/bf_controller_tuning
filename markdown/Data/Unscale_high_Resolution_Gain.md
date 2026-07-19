# Processing of High-Resolution Gains

## Overview

Betaflight provides a **High Resolution Mode** for the Blackbox logger that improves data precision for critical flight signals. It is an optional Blackbox setting that multiplies specific signals by a scaling factor before storing them to preserve decimal precision while using integer storage.

The data is stored as integers. For example, a gyro value of 45.7°/s would lose decimal precision if stored directly as the integer 45. Multiplying by 10 first (resulting in 457) preserves one decimal place while maintaining integer storage.


## Affected Signals

High Resolution Mode scales the following four signals by a factor of **10**:

| Signal Name | Description |
|------------|-------------|
| **gyroADC** | Filtered gyro data (after all digital filters) |
| **gyroUnfilt** | Raw gyro data before filtering |
| **rcCommand** | Processed RC stick commands |
| **setpoint** | Final controller target values | 

## Rescaling for Analysis

To correctly analyze the data (for example FFT analysis or PID tuning), the affected channels must be divided by the same scaling factor. Only then do the values represent real gyro and RC measurements in physical units.

Without this rescaling, all affected values would be 10 times too large and would lead to severe errors in analysis:

- incorrect gyro amplitude → wrong FFT peaks
- incorrect setpoints → distorted tracking analysis
- incorrect RC values → incorrect stick response models
- faulty PID calculations
- Overall, any frequency response analysis would be unusable


## Enabling High Resolution Mode

### Via CLI
```
set blackbox_high_resolution = ON
save
```

Check current setting with:
```
get blackbox_high_resolution
```


## References and Sources

**Blackbox Configuration and Scaling Factor:**
- [src/main/blackbox/blackbox.h](https://github.com/betaflight/betaflight/blob/7f8e4d5b8af17bee89b482d7f4aa1ad40e772496/src/main/blackbox/blackbox.h#L8-L95)  
  Defines the `blackboxConfig_t` structure including the `high_resolution` field.

- [src/main/blackbox/blackbox.c](https://github.com/betaflight/betaflight/blob/7f8e4d5b8af17bee89b482d7f4aa1ad40e772496/src/main/blackbox/blackbox.c#L2312)  
  Scaling factor initialization: `blackboxHighResolutionScale = blackboxConfig()->high_resolution ? 10.0f : 1.0f;`

**Application to Signals:**
- [src/main/blackbox/blackbox.c](https://github.com/betaflight/betaflight/blob/7f8e4d5b8af17bee89b482d7f4aa1ad40e772496/src/main/blackbox/blackbox.c#L1258-L1285)  
  Application of scaling to `gyroADC`, `gyroUnfilt`, `rcCommand` and `setpoint` signals.

### Related Resources

- [Betaflight GitHub Repository](https://github.com/betaflight/betaflight) - Official firmware source code