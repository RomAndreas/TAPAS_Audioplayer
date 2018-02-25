# TAPAS_Audioplayer
This project uses the SDI-TAPAS-Board from Siemens as a power amplifier. Together with a Raspberry Pi Zero, pcm audio files can be played back.

You need to convert the songs to 16Bit unsigned pcm with a sampling-rate of 16kHz and the copy it to the Raspberry Pi Zero 
attached to the TAPAS-Board. The Raspberry Pi is set up with a dietpi-image. 

In further, you need to connect a simple filter (one L, one R and two C's) to the output of the TAPAS power stage and 
then you can connect a Speaker and start playback. 
