# Blackbox Log Data Handling

## Header Information

A Betaflight Blackbox log consists of two logically separate sections: <br>
The **header**, which contains configuration and system information and the **data block**, which contains the recorded sensor values.

The data block alone is useless without the header. It contains only raw numerical values with no context. The header describes how this data must be interpreted. It defines the meaning of each column, the units and scaling factors, the sample rate and the system configuration. For this reason, both sections are read and processed systematically.


## Efficient Loading of Large Log Files in MATLAB

To avoid repeated CSV parsing, the .csv file is converted to a `.mat` file on first load. Later runs load the binary `.mat` file directly, which is much faster for large logs.


## Extracting Parameter Data from the Blackbox Header

The function `extract_header_information()` reads the textual header section of the log file.  
This part of the log contains all relevant Betaflight parameters, such as loop frequencies, filter configurations, logging settings, etc.

Extraction starts at the line containing `frameIntervalI` and ends at `loopIteration`. All lines in this range are parsed and stored in the struct `para`. This creates a clean and structured collection of all metadata needed for downstream analysis.


## Building the Signal Index

The line containing `loopIteration` marks the end of the header and lists all recorded signals in column order as they appear in the .csv file.

This line is parsed to build the index struct `ind`, mapping each signal name to its column index. This enables name-based access to the data matrix instead of hardcoded column numbers.


## Loading the Measurement Data
 
Because the exact number of header lines (`Nheader`) has been determined, MATLAB can reliably read all measurement values by skipping the header and interpreting the following lines as a numerical matrix `data`. It contains one sample per row and one signal per column.

The previously created index struct `ind` determines which column corresponds to which signal.  
This enables name based access, for example:

- `data(:, ind.gyroADC(1))` – Gyro X  
- `data(:, ind.setpoint(3))` – Yaw setpoint  
- `data(:, ind.motor)` – Motor outputs
