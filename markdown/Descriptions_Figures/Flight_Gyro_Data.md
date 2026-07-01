# Flight Gyro Data

This plot gives an overview of the recorded gyro signals for the three axes, roll, pitch and yaw during your flight. You can see the controller setpoints, as well as the unfiltered and filtered gyro signals. This visualizes the differences between the commanded values and the actual sensor data. 

<p align="center">
  <img src="./Images/flight_gyro_data.jpg"
     alt="Original noisy signals"
     width="1000"
     style="float:center; margin-left:10px; margin-right:10px;">
</p>

By looking at the plot, you can, for example, clearly see the [Chirp Signals](../Sinarg/Chirp.md) in the three axes (frequency sweeps). With this log file, you can later tune your drone's PID controllers.
