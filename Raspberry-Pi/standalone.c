#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <sys/wait.h>
#include <inttypes.h>

// declare pin numbers
const int playBut =  26;	// BCM 26 (Pin 37)
const int nextBut = 6;		// BCM 6 (Pin 31) 
	
const int debounce = 60; 	// debounxce-time in ms
const int playDelay = 250; 	// delay between song start and control over stopping it
uint32_t state = 0x000; 

// comment this in if you want to see the current state
//#define DEBUG

// read the play-button
int playPressed(){
	if(digitalRead(playBut) == 0){
		delay(debounce);
		if(digitalRead(playBut) == 0){ 
			return 1;
		}else{
			return 0; 
		} 
	}else{
		delay(debounce); 
		return 0; 
	}
}

// read the next-song button
int nextPressed(){
	if(digitalRead(nextBut) == 0){
		delay(debounce);
		if(digitalRead(nextBut) == 0){
			return 1; 
		}else{
			return 0; 
		}
	}else{
		delay(debounce); 
		return 0; 
	}
}

int main(void){
	wiringPiSetupGpio(); 

	pinMode(playBut, INPUT); 
	pinMode(nextBut, INPUT); 

	pullUpDnControl(playBut, PUD_UP); 
	pullUpDnControl(nextBut, PUD_UP); 

	// start over 
	state = 0x010; // per default, set the state-machine to the first song

	pid_t childPID;
	int statval; 

	while(1){
		#ifdef DEBUG
			printf("state : %x \n\r", state); 
		#endif

		switch(state){
			/*	ALL STATES FOR SONG NUMBER 1 */
			case 0x010:{	/* song 1 selected */
				if(nextPressed()){
					// choose next song
					state = 0x020; 
				}
				if(playPressed()){
					// play song
					state = 0x011;
				}	
				break; 
			}
			case 0x011:{
				// play song number 1; 
				// fork the player in a child process
			        childPID = fork();
 
 			        if(childPID == 0){
                 			// this is the child that is playing the music  
                 			char *exArgs[2] = {"Kompletteinlesen", "./songs/song1.pcm"};
                 			execv ("./Kompletteinlesen", exArgs);
        			}else{
					state = 0x012;
					delay(playDelay);  
				} 
				break;		
				}
			case 0x012:{
				// stop playing song number 1, if play-button is pressed 
				if( waitpid(childPID,NULL,WNOHANG) == 0 ){
					// song is running 
					if(playPressed()){
						// kill child
						kill(childPID, SIGTERM); 	
						// -> return to song 1
						state = 0x010;
					}
				}else{
					printf("playing song 1 ready \n\r");
					state = 0x010;  
				}
				break; 
				}

			/*	ALL STATES FOR SONG NUMBER 2 */
			case 0x020:{	/* song 2 selected */
                                if(nextPressed()){
                                         // choose next song
                                         state = 0x030;
                                 }
                                 if(playPressed()){
                                         // play song
                                         state = 0x021;
                                 }
				break; 
			}
                        case 0x021:{
                                // play song number 2
                                 // fork the player in a child process
                                 childPID = fork();
  
                                 if(childPID == 0){
                                        // this is the child that is playing the music  
                                        char *exArgs[2] = {"Kompletteinlesen", "./songs/song2.pcm"};
                                        execv ("./Kompletteinlesen", exArgs);
                                 }else{
                                        state = 0x022;
					delay(playDelay); 
                                 }
				break;
                                }
                       case 0x022:{
                               // stop playing song number 2
                                if( waitpid(childPID,NULL,WNOHANG) == 0 ){
                                        // song is running 
                                        if(playPressed()){
                                                // kill child
                                                kill(childPID, SIGTERM);
                                                // -> return to song 2
                                                state = 0x020;
                                        }
                                }else{
                                        printf("playing song 2 ready \n\r");
                                        state = 0x020;  
                                }

                                break; 
                                }

			/*	ALL STATES FOR SONG NUMBER 3 */
			case 0x030:{	/* song 3 selected */
                                 if(nextPressed()){
                                         // choose next song
                                         state = 0x040;
                                 }
                                 if(playPressed()){
                                          // play song
                                         state = 0x031;
                                 }
				break;  
			}
                        case 0x031:{
                                // play song number 3
                                // fork the player in a child process
                                childPID = fork();

                                if(childPID == 0){
                                        // this is the child that is playing the music  
                                        char *exArgs[2] = {"Kompletteinlesen", "./songs/song3.pcm"};
                                        execv ("./Kompletteinlesen", exArgs);
                                }else{
                                        state = 0x032;
					delay(playDelay); 
                                }

                                break;
                                }
                        case 0x032:{
                                // stop playing song number 3 
                                if( waitpid(childPID,NULL,WNOHANG) == 0 ){
                                        // song is running 
                                        if(playPressed()){
                                                // kill child
                                                kill(childPID, SIGTERM);
                                                // -> return to song 3
                                                state = 0x030;
                                        }
                                }else{
                                        printf("playing song 3 ready \n\r");
                                        state = 0x030;
                                }
                                break; 
                                }


			/*	ALL STATES FOR SONG NUMBER 4 */	
			case 0x040:{	/* song 4 selected */
                                 if(nextPressed()){
                                         // choose next song
                                         state = 0x050;
                                 }
                                 if(playPressed()){
                                         // play song
                                         state = 0x041;
                                 }
				break; 
			}
                        case 0x041:{
                                // play song number 4
                                // fork the player in a child process
                                childPID = fork();

                                if(childPID == 0){
                                        // this is the child that is playing the music  
                                        char *exArgs[2] = {"Kompletteinlesen", "./songs/song4.pcm"};
                                        execv ("./Kompletteinlesen", exArgs);
                                }else{
                                        state = 0x042;
					delay(playDelay); 
                                }		
                                break;
                                }
                        case 0x042:{
                                // stop playing song number 4 
                                if( waitpid(childPID,NULL,WNOHANG) == 0 ){
                                        // song is running 
                                        if(playPressed()){
                                                // kill child
                                                kill(childPID, SIGTERM);
                                                // -> return to song 4
                                                state = 0x040;
                                        }
                                }else{
                                        printf("playing song 4 ready \n\r");
                                        state = 0x040;
                                }
                                break; 
                                }

			/*	ALL STATES FOR SONG NUMBER 5 */
			case 0x050:{	/* song 5 selected */
                                 if(nextPressed()){
                                         // choose next song
                                         state = 0x010;
                                 }
                                 if(playPressed()){
                                         // play song
                                         state = 0x051;
                                 }
				break;
			}
                        case 0x051:{
                                // play song number 5
                                // fork the player in a child process
                                childPID = fork();
   
                                if(childPID == 0){
                                        // this is the child that is playing the music  
                                        char *exArgs[2] = {"Kompletteinlesen", "./songs/song5.pcm"};
                                        execv ("./Kompletteinlesen", exArgs);
                                }else{
                                        state = 0x052;
					delay(playDelay); 
                                }
                                break;
                                }
                        case 0x052:{
                                // stop playing song number 5 
                                if( waitpid(childPID,NULL,WNOHANG) == 0 ){
                                        // song is running 
                                        if(playPressed()){
                                                // kill child
                                                kill(childPID, SIGTERM);
                                                // -> return to song 5
                                                state = 0x050;
                                        }       
                                }else{  
                                        printf("playing song 5 ready \n\r");
                                        state = 0x050;
                                }       
                                break; 
                                }

			default:{
				printf("!!Statemachine-Error!!\n\r"); 
				printf(" This should never be reached ... \n\r");  
				break; 
			}
		}
	}

	return 0; 
}
