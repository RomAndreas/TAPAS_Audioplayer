# TAPAS_Audioplayer
This project uses the SDI-TAPAS-Board from Siemens as a power amplifier. Together with a Raspberry Pi Zero, pcm audio files can be played back.

You need to convert the songs to 16Bit unsigned pcm with a sampling-rate of 22,05kHz and copy it to the Raspberry Pi Zero 
attached to the TAPAS-Board. The Raspberry Pi is set up with a dietpi-image. 
For audio file conversion, I did a two-step procedure. 
1) normalize the audio file for maximum amplitude input (I used Audacity)
2) convert the normalized file to pcm-format (I used ffmpeg)
```
ffmpeg -i <input-file.wav, .mp3> -f u16le -ac 1 -ar 22050 -acodec pcm_u16le <output-file.pcm>
```
Further, you need to connect a simple filter (one L, one R and two C's) to the output of the TAPAS power stage and 
then you can connect a Speaker and start playback. The schematic for the filter is contained here in the folder "media". 

All the DSP-Code-Project is based on the controlSuite-package from Texas Instruments. 
http://www.ti.com/tool/CONTROLSUITE
One of the example-projects was copied, modified and further example-projects were combined in until this 
firmware was created. 
