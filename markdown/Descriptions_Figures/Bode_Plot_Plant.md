# Bode Plot Plant

The plant Bode plot mainly serves as an informational tool. It gives an overview of how well the physical system has been measured and how reliable the extracted plant model is. 

The **magnitude** and **phase** curves show whether the measurement behaves smoothly and follows the expected system dynamics, while the **coherence** curve is an indicator of measurement quality. High coherence means the data can be trusted, whereas low coherence suggests that noise or limitations in the logging make the results less meaningful in those frequency ranges.

<p align="center">
  <img src="./Images/Bode_Plot_Plant.jpg"
     alt="Original noisy signals"
     width="1000"
     style="float:center; margin-left:10px; margin-right:10px;">
</p>

The plant Bode plot helps the user verify that the system identification worked correctly. It ensures that the controller is being tuned based on solid, trustworthy input data. In this sense, the plot is primarily used to check the validity and consistency of the measurement rather than to directly adjust gains or filter settings.
It's completely normal for the coherence to decrease at higher frequencies. The aim is to achieve good coherence up to 100 Hz. Poor coherence may result from windy weather during the measurement flight.
