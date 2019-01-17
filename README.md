# TAPAS_Audioplayer
This project uses the SDI-TAPAS-Board from Siemens as a power amplifier. Together with a Raspberry Pi Zero, pcm audio files can be played back.

You need to convert the songs to 16Bit unsigned pcm with a sampling-rate of 22,05kHz and copy it to the Raspberry Pi Zero 
attached to the TAPAS-Board. 
The Raspberry Pi is set up with a dietpi-image as follows:

1) format SD card
2) download latest dietpi-image
3) transfer the dietpi-image to the SD-card e.g. by using "Win32DiskImager"
4) enable the spi-device of the raspberry through the following:
a) log in to your raspberry pi
b) execute
```
cd /DietPi
nano config.txt
``` 
change the following from 

```
#-------spi-------------
dtparam=spi=off
```

to

```
#-------spi-------------
dtparam=spi=on
```
and save the changes

5) install the wiringPi-lib on your raspberry:
```
sudo apt-get install wiringpi
```     
     
6) log off and prepare your audio-files

For audio file conversion, I did a two-step procedure. 
1) normalize the audio file for maximum amplitude input (I used Audacity)
2) convert the normalized file to pcm-format (I used ffmpeg)
```
ffmpeg -i <input-file.wav, .mp3> -f u16le -ac 1 -ar 22050 -acodec pcm_u16le <output-file.pcm>
```
Further, you need to connect a simple filter (one L, one R and three C's) to the output of the TAPAS power stage and 
then you can connect a Speaker and start playback. The schematic for the filter is contained here in the folder "media". 

All the DSP-Code-Project is based on the controlSuite-package from Texas Instruments. 
http://www.ti.com/tool/CONTROLSUITE
One of the example-projects was copied, modified and further example-projects were combined in until this 
firmware was created. 

To get the script run in the fashion of autostart, i wrote a little shell-script for starting it : 

autostart.sh:
```
  cd <location-of-binary-and-"songs"-folder>
  ./<compiled-binary-from-autostart.c>
```

made this script executable 
```
chmod a+x autostart.sh
```

and then added it as a cronjob at startup: 
```
root@DietPi:~# crontab -e
```

adding the following line : 
```
@reboot <absolute-path-to-autostart.sh>
```
Now, the little Audioplayer-Demo starts directly after the Raspberry-Pi Zero has finished booting up 
and can be controlled by two pushbuttons connected to the Pi.
