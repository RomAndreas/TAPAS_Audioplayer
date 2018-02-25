#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>       
#include <sys/ioctl.h>   
#include <linux/spi/spidev.h>  
#include <wiringPiSPI.h>
#include <sys/types.h>
#include <sys/stat.h>

#define COMMAND_DATA_CODE 0xBEEF
#define BUFFER_FULL_CODE  0xF001

static uint8_t device = 0; 
static uint8_t mode = 1;
static uint8_t bits = 8;
static uint32_t speed = 1800000;
static uint16_t delay;

int SPIfd;
int PCMfd; 

uint16_t data_out;	// Pi  -> DSP 
uint16_t data_in; 	// DSP -> Pi
unsigned char s_data[2];  

int main(int argc, char** argv){

	// SPI-configuration summary
	printf("SPI-CS.........: %d\n", device);
	printf("SPI-Mode.......: %d\n", mode);
	printf("SPI-worlength..: %d\n", bits);
	printf("SPI-speed......: %d Hz (%d kHz)\n", speed, speed/1000);

	// init SPI interface
	SPIfd = wiringPiSPISetup(device, speed);
	ioctl(SPIfd, SPI_IOC_WR_MODE, &mode); 	
	ioctl(SPIfd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if(SPIfd == -1){
		printf("Error opening the SPI-Device!\n"); 
		exit(1); 
	}

	// get the input file size and allocate a uint8_t array for it 
	// then read in file and do the necessary conversion
        struct stat audioFileInfo; 	
	if(stat(argv[1], &audioFileInfo) == 0){
		printf("Audio input file size: %i Bytes\n", audioFileInfo.st_size); 
	}else{
		printf("Error getting input file information -> exiting progran\n"); 
		exit(1); 
	}

	// open the audio input file
	PCMfd = open(argv[1], O_RDONLY);

	if(PCMfd == -1){
		printf("Error opening the audio input file %s\n", argv[1]); 
		exit(1); 
	}else{
		printf("Audio input file opened successfully\n");
	}

	uint8_t * audioData; 
	audioData = (uint8_t *) malloc(audioFileInfo.st_size * sizeof(uint8_t)); 

	if(audioData == NULL){
		printf("Error getting memory for reading in audio data\n"); 
		exit(1); 
	}

	uint8_t readByte = 0; 
	uint64_t index = 0; 

	read(PCMfd, (void*)&audioData[0], audioFileInfo.st_size); 

	printf("Audio file read in complete ...\n"); 
	
	data_in = 0; 
	uint64_t streamIdx = 0; 

	while(1){
		
		if(data_in == 0){
			// first response
			data_out = 32768;
		       	s_data[1] = (data_out & 0x00FF); 
	       		s_data[0] = ((data_out & 0xFF00) >> 8);        
		}else if(data_in == COMMAND_DATA_CODE){
			// Slave requests data
			if(streamIdx < audioFileInfo.st_size){
				// stream data ... 
				s_data[1] = audioData[streamIdx]; 
				s_data[0] = audioData[streamIdx+1]; 
				streamIdx += 2; 	
			}else{	
				// audio file empty
				data_out = 32768;
				s_data[1] = (data_out & 0x00FF); 
				s_data[0] = ((data_out & 0xFF00) >> 8); 			
				break; 
			}

		}else if(data_in == BUFFER_FULL_CODE){
			// Slave Buffer is full, poll slave until data request
			data_out = 0xFFFF;
			s_data[1] = (data_out & 0x00FF); 
			s_data[0] = ((data_out & 0xFF00) >> 8); 
		}
		
		wiringPiSPIDataRW(device, s_data, 2); 

		data_in = s_data[1]; 
		data_in |= (s_data[0] << 8); 
		
	}

        if(close(PCMfd) != 0){
            printf("ERROR closing the audio file \n");
        }else{
            printf("Audio file closed successfully \n");
        }

	if(close(SPIfd) != 0){
 	    	printf("Error closing the SPI device \n"); 
	}else{
		printf("SPI device closed successfully \n"); 
	}

	free(audioData); 	
}	
