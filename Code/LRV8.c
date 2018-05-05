// to build:
// gcc -o LRV7 LRV7.c -lpigpio -pthread -lbluetooth -L"/usr/sbin" -lmosquitto -lm
#include <netinet/tcp.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <signal.h>
#include <pigpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <time.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mosquitto.h"
#include "LRV6.h"


/* 
These are the state variables of the system. Descriptions of each variable is given below

Parameter:
running - It indicates the mode of control being used 

Possible values:
0 			- indicates that the program should be stopped immediately
1 			- the robot is being controlled using the PC alone
2 (default) - the robot is being controlled by both 
			  the PC and by the remote controller

Parameter:
learnStatus - it indicates whether the robot is learning/has learnt a path

Possible values:
0			- currently not learning and no paths have been learnt (learn mode is yet to be initiated)
1 			- currently learning a path (learn mode is active)
2			- learning has been paused and can be resumed from where it left off (learn mode has paused)
3			- A path has been learnt and stored (learn mode is complete and currently inactive)

Parameter:
repeatStatus - It indicates whether the bot is retracing the learnt path 

Possible values:
0 			- currently not repeating any path	(repeat mode is disabled/inactive)
1 			- currently repeating the stored path (repeat mode is active)
2 		    - repetition has been paused and can be resumed from where it left off	
			  (repeat mode has paused)


NOTE:			  
While the learn and repeat modes have been paused/ are inactive, the robot can be controlled
freely using either the PC or remote controller. If any mode is active, it needs to be stopped/
paused before the bot can be freely controlled. The modes can be changed by taking specific actions
(in the case of the remote controller) or by sending specific commands (in the case of the PC).
These are described in more detail in the main function.


Parameter:
recvd - It indicates whether a message which has been received from the PC has been
		acknowledged

Possible values:
0 			- no message has been received/ message was received and acknowledged
1 			- currently processing the received message

Parameter:
dirCorrection - It is used to ensure that the robot moves in the correct direction.
				It can sometimes happen while using the PID controller that the magnitude
				of the output is accurate but the sign is not (this is possible because the 
				optical encoder used can only measure the speed of rotation and not the 
				direction). In these cases, if dirCorrection is enabled, the polarities of 
				the signals applied to the motors are corrected so that they rotate in the 
				proper direction

Possible values:
0 			- disabled dirCorrection
1 (default)	- enabled dirCorrection

Parameters:
rdir and ldir - they are used in coordination with dirCorrection to set to motor
				directions. rdir is used to control the direction of the right motor
				and ldir is used to control the direction of the left motor

Possible values:
1			- positive voltage must be applied to the motor
2 (default)	- positive voltage must be applied to the motor	

Parameter:
BTenabled     - indicates whether Bluetooth has been enabled and is working properly
				the remote controller can be acknowledged only if BTenabled is 1
				
Possible values:
0 (default)	- Bluetooth is currently disabled/not functioning on the Raspberry Pi
1			- Bluetooth is currently enabled
*/

volatile int running = 2;
volatile int learnStatus = 0;
volatile int repeatStatus = 0;
static volatile int recvd = 0;
int dirCorrection = 1;
int rdir = 2, ldir = 2;
int BTenabled = 0;

/* 
	tick, tock, and joyTick are counters used while learning a path, 
	repeating a path, and measuring time since the joystick switch was 
	pressed down, respectively. The variables tick and tock increment
	by 1 every CMD_TIME_RESOLUTION milliseconds. joyTick increments by
	1 every 250 milliseconds
	
*/
unsigned long int tick = 0, tock = 0, joyTick = 0;

/* 
	msg is a character array to store the command
	received from the PC
*/
char msg[5] = {0};


/*
	cmdArray stores the commands received while the robot is in learn mode. 
	Its size is such that it can store commands worth 10 minutes of time. commands
	are written to it in learn mode and read from it repeat mode
	
	cmd is the variable used while writing to cmdArray
	order is the variable used while reading from cmdArray
*/
char cmdArray[CALC_SIZE] = {0};
char cmd = 0, order = 0;

/*
	The following are arrays used to store the pin numbers associated with 
	GPIO pins connected to the L293D driver, the encoder outputs, and the 
	interrupt handlers associated with the same.
*/
int encoderPins[ENCODER_PINS] = {REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
void* encoderHandler[ENCODER_PINS] = {RLencoderHandler, RRencoderHandler};
int motorDriverPins[4] = {MOTOR_LA, MOTOR_LB, MOTOR_RA, MOTOR_RB};

// counters associated with the Left and Right encoders
volatile int RLEcount = 0; // rear left encoder
volatile int RREcount = 0; // rear right encoder

// RPMs of right and left motors
volatile double rightVel = 0;
volatile double leftVel = 0;

/*
	These are the desired RPMs. When a feedback
	controller is used to control the speed, the controller
	tries to achieve the value stored in these variables.	
*/
int rightTargetRPM = 0;
int leftTargetRPM = 0;

/* 
	error bound for motor speeds
 */
int tol = 2;

int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;

/*
	rarray and larray store the most recently calculated values of the 
	right and left motor RPMs. leftAvg and rightAvg are the averages
	of these arrays.
*/
double leftAvg = 0, rightAvg = 0, rightAvgTemp = 0, leftAvgTemp = 0;
double rarray[AVG_COUNTER] = {0};
double larray[AVG_COUNTER] = {0};
unsigned long int counter = 0;

/*
	left and right are the variables used to set the duty cycles of the left 
	and right motors
*/
double left = STOP;
double right = STOP;

/*
	These are the variables used in feedback control 
	correc is used in the basic feedback controller. It is either added to 
	or subtracted from the duty cycles.
	kp, ki, kd are the parameters of the PID controller
	integralTerm, dervTerm and error are the integral, derivative and error terms
	used in a PID controller. One term each for the left and right motors

*/
double correc = 0.001;
double kp = 0.002, ki = 0.00015, kd = 0.001, kp2 = 1/100, ki2 = 0.00015, kd2 = 0;
double RintegralTerm = 0, LintegralTerm = 0;
double RdervTerm = 0, LdervTerm = 0;
double Rerror = 0, prevRerror = 0;
double Lerror = 0, prevLerror = 0;

FILE *f = NULL;

/*
	mosq is the pointer to the mosquitto structure used to transmitting
	and receive messages to and from the PC
	rc indicates the status of the mosquitto connection. It should be 0
	if the connection is working properly

	Calling mosq after each iteration of the main loop resulted in a lot of overhead and 
	made the Bluetooth response slow so the function mosqLoop will be registered with a timer 
	and then called every couple of seconds
*/
struct mosquitto *mosq;
int rc = 0;	
void mosqLoop(void)
{
	rc = mosquitto_loop(mosq, -1, 1);	
}

int s = 0;
char indicator;
struct sockaddr_rc addr = {0};

/* 
	These are the variables required to estimate the current position and orientation 
	x denotes the x coordinate, y denotes the y coordinate; r, l, c denote the right,
	left, and centre respectively. The 0 indicates the initial value. CALC_SIZE is the
	size of each array. It is calculated in LRV6.h
*/ 

// initial coordinates of the centre, left and right 
double xc0 = 0, yc0 = 0;
double xr0 = DIST/2, yr0 = 0;
double xl0 = -DIST/2, yl0 = 0;

// coordinates while in learn phase
// x and y coordinates of centre, left and right (wheels) at different time instants
double xc_l[CALC_SIZE] = {0};
double yc_l[CALC_SIZE] = {0};
double xr_l[CALC_SIZE] = {0};
double yr_l[CALC_SIZE] = {0};
double xl_l[CALC_SIZE] = {0};
double yl_l[CALC_SIZE] = {0};
double dirx_l[CALC_SIZE] = {0};
double diry_l[CALC_SIZE] = {0};
 
// angular velocities of left and right wheels in natural units
double wr_l[CALC_SIZE] = {0};
double wl_l[CALC_SIZE] = {0};

// coordinates while in repeat phase
// x and y coordinates of centre, left and right (wheels) at different time instants
double xc_r[CALC_SIZE] = {0};
double yc_r[CALC_SIZE] = {0};
double xr_r[CALC_SIZE] = {0};
double yr_r[CALC_SIZE] = {0};
double xl_r[CALC_SIZE] = {0};
double yl_r[CALC_SIZE] = {0};
double dirx_r[CALC_SIZE] = {0};
double diry_r[CALC_SIZE] = {0};

// angular velocities of left and right wheels in natural units
double wr_r[CALC_SIZE] = {0};
double wl_r[CALC_SIZE] = {0};

// unit vector perpendicular to the vector joining the wheels
double dirx0 = 0;
double diry0 = 1;

// path error
double x_error = 0;
double y_error = 0;
double dirx_error = 0;
double diry_error = 0;

double distCorrec = DIST;
int processing = 0;
unsigned long tick_end = 0;
char keywords[15][15] = {"garbage","trash", "plastic","trash","can"};
int check = 0;
int blank = 0;
int camera = 1;
int timing = 0;

// de/activate learn/repeat mode when these go high
int repeatPin = 1;
int learnPin = 1;
int stopPin = 1;


int main(int argc, char** argv)
{		
	char clientid[24];
	unsigned short command[30] = { 0, 0, 1 };
	int bytes_read, x, y, switchVal = 1, prevSwitchVal = 1, prevRepPin = 1, prevLeaPin = 1, prevStopPin = 1;
	
	// dest is the MAC address of the bluetooth module
	char dest[18] = "20:15:04:24:01:30";

	if(argc > 1)
		camera = 0;

	// to generate delays with nanosecond precision, nanosleep is used
	// tim2 is irrelevant to the delay generated
	struct timespec tim, tim3, tim2;
	tim.tv_sec = 0;
	tim.tv_nsec = 150 * 1000 * 1000; // 200 ms
	tim3.tv_sec = 0;
	tim3.tv_nsec = 250 * 1000 * 1000; // 200 ms
	
	// indicator is set as 1 and sent to the HC-05 module to indicate
	// that the bot is ready to receive the next command
	// It is set as 2 and sent to the HC module to ask it to stop/pause
	// transmission
	char indicator;  
	
	// initialize GPIO library
	if (gpioInitialise() < 0)
	{
		printf("Failed to initialize GPIO. Terminating program \n");
		return -1;
	}
	GPIO_pin_init();
	
	// ctr + c handler
	signal(SIGINT, sighandler);
	
	printf("Starting \n");
	
	// Connect to Bluetooth
	// allocate socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	int status = 1;
	// tried to set socket in non-blocking mode, didn't work
	status = fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
	//setsockopt (s, SOL_SOCKET, SO_RCVTIMEO, (char *)&tim3, sizeof(tim3));
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char *) &status, sizeof(int));
	if(status == -1)
	{
		perror("Not able to set non-blocking mode\n");
	}

	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba( dest, &addr.rc_bdaddr );
	
	// connect to server - Bluetooth module
	for(int i = 0; i < 10; i++)
	{
		if(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
		{
			perror("Failed to connect Bluetooth ");
			//return -1;
		}
		else
		{
			printf("Connected to bluetooth successfully. \n");
			BTenabled = 1;
			// enable transmission from Bluetooth module in case it is disabled
			indicator = 1;
			while(write(s, &indicator, 1) == 0);
			break;
		}
		nanosleep(&tim, &tim2);
	}
	BTenabled = 1;
	// load previously stored path data in case any
	f = fopen("Commands.data", "rb");	
	if(f == NULL)
	{
		printf("Couldn't open Commands.data \n");
	}
	else
	{
		fread(cmdArray, sizeof(char), sizeof(cmdArray), f);		
		learnStatus = 3;
		fclose(f);
	}	
	
	// Initialize mosquitto
	mosquitto_lib_init();

	memset(clientid, 0, 24);
	snprintf(clientid, 23, "ID14100");
	mosq = mosquitto_new(clientid, true, 0);
	
	// initialize timer 0 to callback every VEL_STEP milliseconds
	if(gpioSetTimerFunc(0, VEL_STEP, PIDcalcVelocity) != 0)
	{
		printf("Failed  to initialize timer 0\n");
		return -1;
	}
	
	// initialize timer 1. Implements feedback
	if(gpioSetTimerFunc(1, VEL_STEP, PIDControl) != 0)
	{
		printf("Failed  to initialize timer 1\n");
		return -1;
	}
	
	// initialize timer 2. Prints RPMs every 2 seconds
	if(gpioSetTimerFunc(2, 2000, printTimer) != 0)
	{
		printf("Failed  to initialize timer 2\n");
		return -1;
	}
	// initialize timer 3. Match speed of right wheel to that of left
	if(gpioSetTimerFunc(3, VEL_STEP / 2, NULL) != 0)
	{
		printf("Failed  to initialize timer 3\n");
		return -1;
	}
	
	
	if(gpioSetTimerFunc(7, 4000, mosqLoop) != 0)
	{
		printf("Failed  to initialize timer 7\n");
		return -1;
	}
	
	set_motor_speed(STOP, STOP);
	sleep(5);
	
	
	// MAIN LOOP
	
	if(mosq)
	{
		mosquitto_connect_callback_set(mosq, connect_callback);
		mosquitto_message_callback_set(mosq, message_callback);

	    rc = mosquitto_connect(mosq, mqtt_host, mqtt_port, 60);

		mosquitto_subscribe(mosq, NULL, "GSRtest/test1", 0);
		
		while(running)
		{	
			// rc will be 0 if there is no connection problem
			//rc = mosquitto_loop(mosq, -1, 1);
			if(repeatStatus == 4)
				stopRepeatMode();
			
			if(running && rc)
			{
				printf("mosquitto connection error!\n");
				
				sleep(5);
				mosquitto_reconnect(mosq);
			}
			
			if(recvd == 1) 
			{	
				switch(cmd)
				{
					case 's':
					{
						// stop motors
						stop_motors();
						printf("Stop \n");
						break;
					}
					case 'w':
					{
						// move forward
						set_forward();
						printf("Forward \n");
						break;	
					}
					case 'a':
					{
						// turn left
						set_left();
						printf("Left \n");
						break;
					}
					case 'd':
					{
						// turn right
						set_right();
						printf("Right \n");
						break;
					}
					case 'x':
					{
						// go reverse
						set_reverse();
						printf("Reverse \n");
						break;
					}
					case 'z':
					{
						// go reverse
						set_reverse();
						printf("Reverse \n");
						break;
					}
					case 'q':
					{
						// quit program
						stop_motors();
						running = 0;
						printf("Stopping program \n");
						break;
					}
					case 'b':
					{
						// toggle bluetooth control					
						if(running == 1)
							running = 2;
						else if(running == 2)
							running = 1;
						
						printf("Toggled Bluetooth. running = %d \n", running);
						
						break;
					}
					case 'l':
					{
						if(learnStatus == 0 || learnStatus == 3)
						{
							initLearnMode();
						}
						else if(learnStatus == 1 || learnStatus == 2)
						{
							stopLearnMode();
						}
						break;
					}
					case 'r':
					{
						if(learnStatus == 0)
						{
							printf("Haven't learnt any path yet! \n");
						}
						else if(repeatStatus == 0)
						{
							initRepeatMode();
						}
						else if(repeatStatus > 0)
						{
							stopRepeatMode();
						}
						
						break;
					}
					case 'p':
					{
						if(learnStatus == 1)
						{
							pauseLearnMode();
						}
						else if(repeatStatus == 1)
						{
							pauseRepeatMode();
						}
						else if(repeatStatus == 2)
						{
							resumeRepeatMode();
						}
						break;
					}
					case 'o':
					{
						if(learnStatus == 2)
						{
							resumeLearnMode();
						}
						break;
					}
					default:
						printf("Unrecognized command \n");
				}


			}
			
			if((running == 2) && (BTenabled == 1))
			{
				//gpioSetTimerFunc(5, 5000, BTdisc);
				int writer = 1;
				do
				{
					if(timing == 1)
						gpioSetTimerFunc(6, 250, NULL);
					//read(s, command, sizeof(command));
					//int writer = 1;
					indicator = 1;
					setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char *) &writer, sizeof(int));
					write(s, &indicator, 1);
					//nanosleep(&tim3, &tim2);
					bytes_read = read(s, command, 2*6);
					
				}
				while(bytes_read != 2*6 && running == 2);
				check = 3;
				
				if(timing == 1)
					gpioSetTimerFunc(6, 250, switchTick);
				
                //setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char*)&writer, sizeof(int));
				bytes_read = read(s, command, sizeof(command));
				//setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (char *) &writer, sizeof(int));
				printf("%d \n", bytes_read);
				prevSwitchVal = switchVal;
				prevLeaPin = learnPin;
				prevRepPin = repeatPin;
				prevStopPin = stopPin;
				x = command[0];
				y = command[1];
				switchVal = command[2];
				learnPin = command[3];
				repeatPin = command[4];
				stopPin = command[5];
				
				gpioSetTimerFunc(5, 1500, NULL);
							
				if(x > XM || x < 0 || y > YM || y < 0 || switchVal > 1 || switchVal < 0)
				{
					stop_motors();
					//indicator = 2;
					//while(write(s, &indicator, 1) == 0);
					nanosleep(&tim, &tim2);
					check = 0;
				}
				
				if( check == 3 ) 
				{	
					if(prevRepPin == 1 && repeatPin == 0)
					{
						if(repeatStatus == 0)
						{
							initRepeatMode();
						}
						else if(repeatStatus < 4)
						{
							stopRepeatMode();
						}
					}
					if(prevLeaPin == 1 && learnPin == 0)
					{
						if(learnStatus == 0 || learnStatus == 3)
						{
							initLearnMode();
						}
						else
						{
							stopLearnMode();
						}
					}
					if(prevStopPin == 1 && stopPin == 0)
					{
						running = 0;
					}
						
					
					if(repeatStatus == 0 && recvd == 0)
					{
							printf("x: %d y: %d sw: %d, l: %d, r: %d, s: %d \n", x, y, switchVal, learnPin, repeatPin, stopPin);
							printf("Valid values \n");
						if(x > XM || x < 0 || y > YM || y < 0 || switchVal > 1 || switchVal < 0)
						{
							stop_motors();
							indicator = 2;
							while(write(s, &indicator, 1) == 0);
							nanosleep(&tim, &tim2);
						}
						else if((x - XC)*(x - XC) + (y - YC)*(y - YC) < RAD* RAD)
						{
							// stop 
							
							cmd = 's';
							stop_motors();
							printf("BT: stop \n");
						}
						else if( y - x >= 0 )
						{
							if(y + x >= YM)
							{
								// forward
								 
								cmd = 'w';
								set_forward();;
								printf("BT: forward \n");
							}
							else
							{
								// left
								 
								cmd = 'a';
								set_left();;
								printf("BT: left \n");
							}
						}
						else
						{
							if(y + x >= YM )
							{
								// right
								 
								cmd = 'd';
								set_right();
								printf("BT: right \n");
							}
							else
							{
								// reverse
								cmd = 'x';
								set_reverse();
								printf("BT: reverse \n");
							}
						}
					}
				}
				check = 0;
				//indicator = 1;
				//while((write(s, &indicator, 1) == 0) && running == 2);
			}
			recvd = 0;
			
			// generate delay with time struct values in &tim. &tim2 is a dummy
			nanosleep(&tim , &tim2);
		}
		
		mosquitto_destroy(mosq);
	}
	
	// Clean up
	mosquitto_lib_cleanup();
	stop_motors();
	sleep(5);

	if(BTenabled == 1)
	{
		// tell hc module to stop transmitting
		indicator = 2;
		while(write(s, &indicator, 1) == 0);
	
		// close connection	
		close(s);	
		printf("Closed connection \n");
	}
	
	// terminate GPIO library
	gpioTerminate();
}


void BTdisc(void)
{
	close(s);
	sleep(1);
	BTenabled = 0;
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	if(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
	{
		perror("Failed to connect Bluetooth ");
		gpioSetTimerFunc(5, 500, NULL);
		stop_motors();
		sleep(5);
		running = 1;
		close(s);
		//return -1;
	}
	else
	{
		printf("Connected to bluetooth successfully. \n");
		BTenabled = 1;
		running = 2;
		// enable transmission from Bluetooth module in case it is disabled
		indicator = 1;
		while(write(s, &indicator, 1) == 0);
	}
	

}

// initialize GPIO pins
void GPIO_pin_init(void)
{
	for(int i = 0; i < ENCODER_PINS; i++)
	{
		gpioSetMode(encoderPins[i], PI_INPUT);
		gpioSetAlertFunc(encoderPins[i], encoderHandler[i]);
		gpioGlitchFilter(encoderPins[i], 1500); 
		
		// state must persist for at least 1.5 milliseconds to recognize the state change
	}	
	
	for(int i = 0; i < DRIVER_PINS; i++)
	{
		gpioSetMode(motorDriverPins[i], PI_OUTPUT);
		gpioSetPWMfrequency(motorDriverPins[i], FREQ );	
		gpioSetPWMrange(motorDriverPins[i] , RANGE);
	}
}

// Rear Left Encoder interrupt handler
void RLencoderHandler(void)
{
	RLEcount++;
}

// Rear Right Encoder interrupt handler
void RRencoderHandler(void)
{
	RREcount++;
}

//signal handler for ctrl + c (SIGINT)
void sighandler(int dummy)
{
	running = 0;
}

// callback function which calculates velocities and other quantities required for PID control
void PIDcalcVelocity(void)
{
	prevRerror = -rightAvg + rightTargetRPM;
	prevLerror = -leftAvg + leftTargetRPM;
	
	rightVel = (((double) RREcount) / (2 * RSLOTS)) * (60 * 1000 / VEL_STEP); 
	leftVel = (((double) RLEcount) / (2 * LSLOTS)) * (60 * 1000 / VEL_STEP);
	
	RLEcount = 0;
	RREcount = 0;
	
	rarray[counter] = rightVel;
	larray[counter] = leftVel;
	
	rightAvgTemp = 0; 
	leftAvgTemp = 0;
	
	for(int i = 0; i < AVG_COUNTER; i++)
	{
		rightAvgTemp += rarray[i];
		leftAvgTemp += larray[i];
	}
	
	rightAvg = rightAvgTemp / AVG_COUNTER;
	leftAvg = leftAvgTemp / AVG_COUNTER;
	
	Rerror = -rightAvg + rightTargetRPM;
	Lerror = -leftAvg + leftTargetRPM;
	
	RintegralTerm += ki * Rerror;
	LintegralTerm += ki * Lerror;
	
	RdervTerm = kd * (Rerror - prevRerror);
	LdervTerm = kd * (Lerror - prevLerror);
	
	counter++;
	if(counter == AVG_COUNTER)
		counter = 0;	
}

void printTimer(void)
{
	double span = 0;
	
	if(processing == 0)
	{
		printf("right velocity (RPM): %f \n", rightAvg);
		printf("left velocity (RPM): %f \n", leftAvg);	
		
		
		if(repeatStatus == 10)
		{
			printf("x error: %f \n", y_error);
			printf("y error: %f \n", x_error);	
			printf("x dir error: %f \n", dirx_error);
			printf("y dir error: %f \n", diry_error);			
		}
		
		if(learnStatus == 1 || learnStatus == 2)
		{
			printf("x pos: %f \n", xc_l[tick]);
			printf("y pos: %f \n", yc_l[tick]);	
			
			span = (xl_l[tick] - xr_l[tick]) * (xl_l[tick] - xr_l[tick]);
			span += (yl_l[tick] - yr_l[tick]) * (yl_l[tick] - yr_l[tick]);
			span /= DIST;
			span /= DIST;
			
			// span should be approximately 1
			printf(" dist bw wheels: %f \n", span);
		}
	}
}

void correctVelocity(void)
{	
	if((rightAvg - rightTargetRPM) > tol)
	{
		right = right - correc;
		if(right < -1)
			right = -1;
	}
	else if((-rightAvg + rightTargetRPM) > tol)
	{
		right = right + correc;
		if(right > 1)
			right = 1;
	}
		
	if((leftAvg - leftTargetRPM) > tol)
	{
		left = left - correc;
		if(left < -1)
			left = -1;
	}
	else if((-leftAvg + leftTargetRPM) > tol)
	{
		left = left + correc;
		if(left > 1)
			left = 1;
	}
	
	set_motor_speed(left, right);	
}

void PIDControl(void)
{	
	// PID control is implemented only if the target RPMs are greater than 80 
	// if less than 20, the right and left variables need to be set in the program
	
	if(rightTargetRPM >= 20)
	{
		right = kp * Rerror + RintegralTerm + RdervTerm;
		if(right < -1)
			right = -1;
		if(right > 1)
			right = 1;
	}
    
	if(leftTargetRPM >= 20)
	{
		left = kp * Lerror + LintegralTerm + LdervTerm;
		if(left < -1)
			left = -1;
		if(left > 1)
			left = 1;
	}
	
	set_motor_speed(right, right);	
}

void speedMatch(void)
{
	if((rightAvg > leftAvg + tol) || (rightAvg < leftAvg - tol))
	{
		if(right >= 0)
			right = right + kp2 * (leftVel - rightVel);
		else
			right = right - kp2 * (leftVel - rightVel);		
	}

	if(right < -1)
		right = -1;
	if(left < -1)
		left = -1;
	if(right > 1)
		right = 1;
	if(left > 1)
		left = 1;
	
}
int set_motor_speed(double left_duty, double right_duty)
{	
	if(dirCorrection == 1)
	{
		if((right_duty < 0 && (rdir == 1)) || (right_duty > 0 && (rdir == 2)))
			right_duty = -1 * right_duty;
		if((left_duty < 0 && (ldir == 1)) || (left_duty > 0 && (ldir == 2)))
			left_duty = -1 * left_duty;
	}		
	
	// scale inputs from the range [-1, 1] to [0, 1]
	left_duty = 0.5 + left_duty / 2;
	right_duty = 0.5 + right_duty / 2;
	
	// scale [0, 1] to [0, RANGE] - As bidirectional control is used, 0.5 corresponds to 0 speed (0 average voltage)
	// Control B must be the negation of Control B
	// Possible improvement - use hardware to produce the negation instead of coding it
	la_pwm = (int) (RANGE * left_duty);
	lb_pwm = (int) (RANGE * (1 - left_duty));
	ra_pwm = (int) (RANGE * right_duty);
	rb_pwm = (int) (RANGE * (1 - right_duty));
	
	// slightly faster version of above, with no error handling
	gpioPWM(MOTOR_LA, la_pwm);
	gpioPWM(MOTOR_LB, lb_pwm);
	gpioPWM(MOTOR_RA, ra_pwm);
	gpioPWM(MOTOR_RB, rb_pwm);
	
	return 0;
}


void connect_callback(struct mosquitto *mosq, void *obj, int result)
{
	printf("connect callback, rc=%d\n", result);
}

void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
	strcpy(msg, message->payload);
	//printf("got message '%.*s' for topic '%s'\n", message->payloadlen, (char*) message->payload, message->topic);
	cmd = msg[0];
	recvd = 1;
}

void set_forward(void)
{
	rightTargetRPM = ACTIVE_RPM;
	leftTargetRPM = ACTIVE_RPM;
	rdir = 2;
	ldir = 2;
}

void set_reverse(void)
{
	rightTargetRPM = ACTIVE_RPM;
	leftTargetRPM = ACTIVE_RPM;
	rdir = 1;
	ldir = 1;
}

void set_right(void)
{
	rightTargetRPM = ACTIVE_RPM;
	leftTargetRPM = ACTIVE_RPM;
	rdir = 1;
	ldir = 2;
}

void set_left(void)
{
	rightTargetRPM = ACTIVE_RPM;
	leftTargetRPM = ACTIVE_RPM;
	rdir = 2;
	ldir = 1;
}

void stop_motors(void)
{
	rightTargetRPM = 0;
	leftTargetRPM = 0;
	right = STOP;
	left = STOP;
	//LintegralTerm = 0;
	//RintegralTerm = 0;
}

// timer to record duration of joystick switch press
void switchTick(void)
{
	joyTick++;	
}

// ticks every x milliseconds
void learnTimer(void)
{
	cmdArray[tick] = cmd;
	tick++;
	tick_end = tick;
	
	// calculate and store position and orientation of the robot
	// calculate the new direction vector and new coordinates
	wr_l[tick] = (2 * rdir - 3) * 2 * PI * rightAvg / 60;
	wl_l[tick] = (2 * ldir - 3) * 2 * PI * leftAvg / 60;
	
	dirx_l[tick] = -(yr_l[tick - 1] - yl_l[tick - 1]) / DIST;
	diry_l[tick] =  (xr_l[tick - 1] - xl_l[tick - 1]) / DIST;
	
	xr_l[tick] = xr_l[tick - 1] + (RADIUS * wr_l[tick] * dirx_l[tick] * CMD_TIME_RESOLUTION) / 1000;	
	yr_l[tick] = yr_l[tick - 1] + (RADIUS * wr_l[tick] * diry_l[tick] * CMD_TIME_RESOLUTION) / 1000;
	
	xl_l[tick] = xl_l[tick - 1] + (RADIUS * wl_l[tick] * dirx_l[tick] * CMD_TIME_RESOLUTION) / 1000;	
	yl_l[tick] = yl_l[tick - 1] + (RADIUS * wl_l[tick] * diry_l[tick] * CMD_TIME_RESOLUTION) / 1000;
	
	xc_l[tick] = (xl_l[tick] + xr_l[tick]) / 2;
	yc_l[tick] = (yl_l[tick] + yr_l[tick]) / 2;
	
	distCorrec = (xr_l[tick] - xc_l[tick]) * (xr_l[tick] - xc_l[tick]);
	distCorrec += (yr_l[tick] - yc_l[tick]) * (yr_l[tick] - yc_l[tick]);
	distCorrec = sqrt(distCorrec);
	
	xr_l[tick] = (xr_l[tick] - xc_l[tick]) * DIST / (2 * distCorrec) + xc_l[tick];
	yr_l[tick] = (yr_l[tick] - yc_l[tick]) * DIST / (2 * distCorrec) + yc_l[tick];
	
	distCorrec = (xl_l[tick] - yc_l[tick]) * (xl_l[tick] - xc_l[tick]);
	distCorrec += (yl_l[tick] - xc_l[tick]) * (yl_l[tick] - xc_l[tick]);
	distCorrec = sqrt(distCorrec);
	
	xl_l[tick] = (xl_l[tick] - xc_l[tick]) * DIST / (2 * distCorrec) + xc_l[tick];
	yl_l[tick] = (yl_l[tick] - yc_l[tick]) * DIST / (2 * distCorrec) + yc_l[tick];
	
}

void repeatTimer(void)
{
	order = cmdArray[tock];
	
	// for every 50 tocks = , take a pic and store it
	if(tock % 50 == 0 && tock > 0 && processing == 0 && camera == 1)
	{
		processing = 1;
		stop_motors();
		sleep(3);
		printf("Capturing and Analyzing image. This will take some time \n");
		
		
		sleep(3);
		system("fswebcam pic.jpg");
		sleep(2);
		printf("webcam step over \n");
		system("python /home/pi/Testing/imageClassifier/classify_image.py --image=/home/pi/Testing/pic.jpg --model_dir=/home/pi/Testing/imageClassifier/imagenet/ >classified.txt"); 
		sleep(2);
		
		// see if result contains keyword		
		// send coordinates if found
		char msg[20] = {0};
		char *buffer = 0;
		long length;
		FILE *fi = fopen ("garb.txt", "rb");
		
		char found = 0;
		
		if(fi)
		{
		  fseek(fi, 0, SEEK_END);
		  length = ftell(fi);
		  fseek(fi, 0, SEEK_SET);
		  buffer = malloc(length);
		  if(buffer)
		  {
			fread(buffer, 1, length, fi);
		  }
		  fclose(fi);
		}

		if(buffer)
		{
			for(int k = 0; k < 15; k++)
			{
				if(keywords[k][0] != 0)
				{
					printf("%s \n", keywords[k]);

					if(strstr(buffer, keywords[k]) != NULL)
					{
						strncpy(msg, "Found Garbage", sizeof(msg));
						printf("%s: %s \n", msg, keywords[k]);
					}
				}
			}
		}
		processing = 0;
				
	}
	
	if(processing == 0)
	{	
		switch(order)
		{
			case 's':
			{
				// stop motors	
				stop_motors();
				printf("Stop \n");
				break;
			}
			case 'w':
			{
				// move forward
				set_forward();
				printf("Forward \n");
				break;	
			}
			case 'a':			
			{
				// turn left
				set_left();
				printf("Left \n");
				break;
			}
			case 'd':
			{
				// turn right
				set_right();
				printf("Right \n");
				break;
			}
			case 'z':
			{
				// move backward
				set_reverse();
				printf("Reverse \n");
				break;
			}
			case 'x':
			{
				// move backward
				set_reverse();
				printf("Reverse \n");
				break;
			}
			case 'e':
			{
				// end repeat sequence
				stop_motors();
				//printf("Completed REPEAT phase, entering NORMAL mode \n");
				repeatStatus = 4;
				//stopRepeatMode();
				
			}
			default:
			{
				// for commands other than w a s d x z, do nothing
				break;
			}
		}
		
			tock++;
		
		/*if(tock >= tick_end)
		{
			stop_motors();
			printf("Completed REPEAT phase, entering NORMAL mode \n");
			
			stopRepeatMode();
		}*/
		
		wr_r[tock] = (2 * rdir - 3) * 2 * PI * rightAvg / 60;
		wl_r[tock] = (2 * ldir - 3) * 2 * PI * leftAvg / 60;
		
		dirx_r[tock] = -(yr_r[tock - 1] - yl_r[tock - 1]) / DIST;
		diry_r[tock] =  (xr_r[tock - 1] - xl_r[tock - 1]) / DIST;
		
		xr_r[tock] = xr_r[tock - 1] + (RADIUS * wr_r[tock] * dirx_r[tock] * CMD_TIME_RESOLUTION) / 1000;	
		yr_r[tock] = yr_r[tock - 1] + (RADIUS * wr_r[tock] * diry_r[tock] * CMD_TIME_RESOLUTION) / 1000;
		
		xl_r[tock] = xl_r[tock - 1] + (RADIUS * wl_r[tock] * dirx_r[tock] * CMD_TIME_RESOLUTION) / 1000;	
		yl_r[tock] = yl_r[tock - 1] + (RADIUS * wl_r[tock] * diry_r[tock] * CMD_TIME_RESOLUTION) / 1000;
		
		xc_r[tock] = (xl_r[tock] + xr_r[tock]) / 2;
		yc_r[tock] = (yl_r[tock] + yr_r[tock]) / 2;
		
		distCorrec = (xr_r[tock] - xc_r[tock]) * (xr_r[tock] - xc_r[tock]);
		distCorrec += (yr_r[tock] - yc_r[tock]) * (yr_r[tock] - yc_r[tock]);
		distCorrec = sqrt(distCorrec);
		
		xr_r[tock] = (xr_r[tock] - xc_r[tock]) * DIST / (2 * distCorrec) + xc_r[tock];
		yr_r[tock] = (yr_r[tock] - yc_r[tock]) * DIST / (2 * distCorrec) + yc_r[tock];
		
		distCorrec = (xl_r[tock] - yc_r[tock]) * (xl_r[tock] - xc_r[tock]);
		distCorrec += (yl_r[tock] - xc_r[tock]) * (yl_r[tock] - xc_r[tock]);
		distCorrec = sqrt(distCorrec);
		
		xl_r[tock] = (xl_r[tock] - xc_r[tock]) * DIST / (2 * distCorrec) + xc_r[tock];
		yl_r[tock] = (yl_r[tock] - yc_r[tock]) * DIST / (2 * distCorrec) + yc_r[tock];
		
		x_error = xc_r[tock] - xc_l[tock];
		y_error = yc_r[tock] - yc_l[tock];
		dirx_error = dirx_r[tock] - dirx_l[tock];
		diry_error = diry_r[tock] - diry_l[tock];
	}
}

// Initiates a new Learn sequence, previously stored data in the command array will be erased
// Stops currently active repeat sequences
void initLearnMode(void)
{
	// if currently in a repeat sequence, stop it
	if(repeatStatus > 0)
		stopRepeatMode();
	
	// Begin learn mode
	learnStatus = 1;
	
	// stop motors	
	stop_motors();
	
	printf("\n");
	printf("Initiating LEARN mode \n");
	printf("Clearing arrays... \n");
	
	for(int i = 0; i < CALC_SIZE; i++)
	{
		cmdArray[i] = 0; 
		
		xl_l[i] = 0;
		xr_l[i] = 0;
		yl_l[i] = 0;
		yr_l[i] = 0;
		wr_l[i] = 0;
		wl_l[i] = 0;
		xc_l[i] = 0;
		yc_l[i] = 0;
		dirx_l[i] = 0;
		diry_l[i] = 0;
		
	}
	
	printf("Initializing timer and variables \n");
	
	tick = 0; tick_end = 0;
	
	xl_l[0] = xl0;
	xr_l[0] = xr0;
	yl_l[0] = yl0;
	yr_l[0] = yr0;
	xc_l[0] = xc0;
	yc_l[0] = yc0;
	dirx_l[0] = dirx0;
	diry_l[0] = diry0;
	
	
	gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, learnTimer);
	
	printf("Done \n");
	
}

void initRepeatMode(void)
{
	stop_motors();
	
	if(learnStatus == 0)
	{
		printf("Haven't learnt anything yet");
	}
	else 
	{
		if(learnStatus < 3)
		{
			pauseLearnMode();
		}
		
		printf("\n");
		printf("Initiating REPEAT mode \n");
		printf("Clearing arrays... \n");
	
		for(int i = 0; i < CALC_SIZE; i++)
		{	
			xl_r[i] = 0;
			xr_r[i] = 0;
			yl_r[i] = 0;
			yr_r[i] = 0;
			wr_r[i] = 0;
			wl_r[i] = 0;
			xc_r[i] = 0;
			yc_r[i] = 0;
			dirx_r[i] = 0;
			diry_r[i] = 0;			
		}
		
		xl_r[0] = xl0;
		xr_r[0] = xr0;
		yl_r[0] = yl0;
		yr_r[0] = yr0;
		xc_r[0] = xc0;
		yc_r[0] = yc0;
		dirx_r[0] = dirx0;
		diry_r[0] = diry0;
	
		printf("Initializing timer... \n");
		repeatStatus = 1;
		tock = 0;
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, repeatTimer);
		
		printf("Done \n");
	}
}

void pauseLearnMode(void)
{
	stop_motors();
	if(learnStatus > 0 && learnStatus < 3)
	{
		learnStatus = 2;
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, NULL);
		printf("Paused learn mode \n");
	}
}

void pauseRepeatMode(void)
{
	stop_motors();
	if(repeatStatus > 0)
	{
		repeatStatus = 2;
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, NULL);
		printf("Paused repeat mode \n");
	}
}

void resumeLearnMode(void)
{
	stop_motors();
	if(repeatStatus == 1)
		pauseRepeatMode();
	
	if(learnStatus == 2)
	{
		printf("Re-Initializing timer in Learn mode \n");
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, learnTimer);
		learnStatus =  1;
		printf("Done \n");
	}
	else
	{
		printf("Learn mode has not yet been initialized or has already ended. learnStatus = %d \n", learnStatus);
	}
}

void resumeRepeatMode(void)
{
	stop_motors();
	
	if(learnStatus == 0)
	{
		printf("Haven't learnt anything yet");
	}
	else if(repeatStatus == 2)
	{
		if(learnStatus < 3)
		{
			pauseLearnMode();
		}
		
		printf("\n");
		printf("resuming REPEAT mode \n");
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, repeatTimer);
		repeatStatus = 1;
		
		printf("Done \n");
	}
	else if(repeatStatus != 2)
	{
		printf("Program has either completed repetition or not started it yet. repeatStatus = %d \n", repeatStatus);
	}
}

void stopLearnMode(void)
{
	stop_motors();
	
	if(learnStatus == 0)
	{
		printf("Haven't learnt anything yet");
	}
	else if(learnStatus < 3)
	{	
		printf("\n");
		printf("Stopping learn mode \n");
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, NULL);
		
		// set end indicator
		cmdArray[tick] = 'e';
		
		tick = 0;
		learnStatus = 3;
		
		printf("Saving data... \n");
		
		f = fopen("Commands.data", "wb");
		if(f == NULL)
		{
			printf("Couldn't open file for writing data \n");
			printf("Saved data will still be accessible in cmdArray until the program ends\n");
		}
		else
		{
			fwrite(cmdArray, sizeof(char), sizeof(cmdArray), f);
			fclose(f);
			printf("Done \n");
		}
		
		f = fopen("xc.data", "wb");
		if(f == NULL)
		{
			printf("Couldn't open file for writing data \n");
			//printf("Saved data will still be accessible in cmdArray until the program ends\n");
		}
		else
		{
			fwrite(xc_l, sizeof(char), sizeof(xc_l), f);
			fclose(f);
			printf("Done \n");
		}
		
		f = fopen("yc.data", "wb");
		if(f == NULL)
		{
			printf("Couldn't open file for writing data \n");
			//printf("Saved data will still be accessible in cmdArray until the program ends\n");
		}
		else
		{
			fwrite(yc_l, sizeof(char), sizeof(yc_l), f);
			fclose(f);
			printf("Done \n");
		}
	}
}

void stopRepeatMode(void)
{
	stop_motors();
	
	if(learnStatus == 0)
	{
		printf("Haven't learnt anything yet");
	}
	else
	{
		if(learnStatus < 3)
		{
			pauseLearnMode();
		}
		
		printf("\n");
		printf("Stopping REPEAT mode \n");
		gpioSetTimerFunc(4, CMD_TIME_RESOLUTION, NULL);
		repeatStatus = 0;
		printf("Done \n");
	}
}

