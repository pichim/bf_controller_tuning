# bf_controller_tuning

WIP

A huge ass script to analyse **bbl** flight logs and tune linear filters and pids offline.

### Required Matlab Toolboxes

- MATLAB
- Control System Toolbox
- Signal Processing Toolbox
- Statistics and Machine Learning Toolbox (dono why atm)

I might try to reduce the dependencies in the future.

### Running the Example Script

1. Get the latest artefacts from the blackbox-log-viewer https://github.com/pichim/blackbox-log-viewer/actions/runs/8053874808 and install it.
2. Open the file **20240225_apex5_00.bbl** and save it in the directory 20240225 as **20240225_apex5_00.bbl.csv**.

## Betaflight PR to perform the Chirp Experiments

The PR can be flashed and tested with `#13105` in the betaflight-configurator.

 Chirp signal generator as flight mode `#13105`: </br>
https://github.com/betaflight/betaflight/pull/13105

Branch on pichim's fork: </br>
https://github.com/pichim/betaflight/tree/dev_chirp

## Custom Matlab Libary

To have the example running, place the libary in the same directory as `bf_controller_tunning`: </br>
https://github.com/pichim/bf_function_libary

## Custom BBLV Artefacts

Added chirp parameters to header `#658`: </br>
https://github.com/betaflight/blackbox-log-viewer/pull/658

Branch on pichim's fork: </br>
https://github.com/pichim/blackbox-log-viewer/tree/dev_chirp