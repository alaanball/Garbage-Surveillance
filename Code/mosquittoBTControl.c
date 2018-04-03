#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pigpio.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "mosquitto.h"

#define mqtt_host "test.mosquitto.org"
#define mqtt_port 1883

// These are the GPIO numbers, not the pin numbers - see the rpi schematic for the mapping between the two
#define MOTOR_LA 5	  // pin 29 - left side, 6th from bottom	
#define MOTOR_RA 19   // pin 35 - left side, 3rd from bottom
#define MOTOR_LB 6    // pin 31 - left side, 5th from bottom
#define MOTOR_RB 26   // pin 37 - left side, 2nd from bottom

#define FREQ 5000

#define XM 1024
#define YM 1024
#define XC 512
#define YC 512
#define RAD 150
#define YTOPTHRESH 720
#define YBOTTHRESH 280

#define XLEFTTHRESH 280
#define XRIGHTTHRESH 720

#define RANGE 255

#define FORWARD 0.5
#define BACKWARD -0.5
#define STOP 0

static volatile int run = 1;							// set as 0 when ctrl + c is pressed; program then exits main while loop
static volatile int recvd = 0;

void GPIO_pin_init(void);									// sets GPIO pin modes, frequencies, and states
int set_motor_speed(float left_duty, float right_duty); 	// argument range: -1 to 1, -1 = max speed, reverse; 1 = max speed, forward

int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;
int motorPins[4] = {MOTOR_LA, MOTOR_LB, MOTOR_RA, MOTOR_RB};
char msg[5] = {0};
char cmd = 0;

int command = 0;

//signal handler for ctrl + c (SIGINT)
void sighandler(int dummy)
{
	run = 0;
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

int main(int argc, char *argv[])
{
	uint8_t reconnect = true;
	char clientid[24];
	char msg[5] = "test";
	struct mosquitto *mosq;
	int rc = 0;
	struct sockaddr_rc addr = { 0 };
	short command[2] = { 0 };
	int s, bytes_read, x, y; 
	char dest[18] = "20:15:04:24:01:30";
	int BTenabled = 0;

	// set as 1 and send to hc module to start transmission; set as 2 and send to hc module to stop transmission
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
	
	// allocate socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	
	// set the connection parameters (who to connect to)
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba( dest, &addr.rc_bdaddr );
	
	// connect to server - Bluetooth module
	if(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
	{
		perror("Failed to connect");
		//return -1;
	}
	else
	{
		printf("Connected to bluetooth successfully. \n");
		BTenabled = 1;
		// enable transmission from Bluetooth module in case it is disabled
		indicator = 1;
		while(write(s, &indicator, 1) == 0);
	}
	mosquitto_lib_init();

	memset(clientid, 0, 24);
	snprintf(clientid, 23, "ID14170");
	mosq = mosquitto_new(clientid, true, 0);

	if(mosq)
	{
		mosquitto_connect_callback_set(mosq, connect_callback);
		mosquitto_message_callback_set(mosq, message_callback);

	    rc = mosquitto_connect(mosq, mqtt_host, mqtt_port, 60);

		mosquitto_subscribe(mosq, NULL, "GSRtest/test1", 0);

		while(run)
		{
			rc = mosquitto_loop(mosq, -1, 1);
			
			if(run && rc)
			{
				printf("mosquitto connection error!\n");
				
				sleep(5);
				mosquitto_reconnect(mosq);
			}
			
			if(recvd == 1) 
			{				
				switch(cmd)
				{
					case 'j':
					{
						// stop motors
						set_motor_speed(STOP, STOP);
						printf("Stop");
						break;
					}
					case 'w':
					{
						// move forward
						set_motor_speed(FORWARD, FORWARD);
						printf("Forward");
						break;	
					}
					case 'a':
					{
						// turn left
						set_motor_speed(BACKWARD, FORWARD);
						printf("Left");
						break;
					}
					case 'd':
					{
						// turn right
						set_motor_speed(FORWARD, BACKWARD);
						printf("Right");
						break;
					}
					case 's':
					{
						// move backward
						set_motor_speed(BACKWARD, BACKWARD);
						printf("Reverse");
						break;
					}
					case 'p':
					{
						// stop program
						run = 0;
						set_motor_speed(STOP, STOP);
						printf("Stopping program");
						break;
					}
					case 'b':
					{
						// toggle bluetooth control
						
						if(run == 1)
							run = 2;
						else if(run == 2)
							run = 1;
						
						printf("Toggled Bluetooth. run = %d", run);
						
						break;
					}
					default:
						printf("Unrecognized command \n");
				}
			
				recvd = 0;
			}

			if((run == 2) && (BTenabled == 1))
			{
				// implement bluetooth control
				bytes_read = read(s, command, sizeof(command));
				if( bytes_read > 0 ) 
				{		
					x = command[0];
					y = command[1];
					
					//printf("x: %d y: %d \n", x, y);
					
					if(x > XM || x < 0 || y > YM || y < 0); // do nothing if garbage values
					else if((x - XC)*(x - XC) + (y - YC)*(y - YC) < RAD* RAD)
					{
						// stop 
						set_motor_speed(STOP, STOP);
						printf("stop \n");
					}
					else if( y - x >= 0 )
					{
						if(y + x >= YM)
						{
							// up
							set_motor_speed(FORWARD, FORWARD);
							printf("forward \n");
						}
						else
						{
							// left
							set_motor_speed(BACKWARD, FORWARD);
							printf("left \n");
						}
					}
					else
					{
						if(y + x >= YM )
						{
							// right
							set_motor_speed(FORWARD, BACKWARD);
							printf("right \n");
						}
						else
						{
							// down
							set_motor_speed(BACKWARD, BACKWARD);
							printf("reverse \n");
						}
					}
				}		
				sleep(0.5);
			}

		}
		mosquitto_destroy(mosq);
	}

	mosquitto_lib_cleanup();
	
	if(BTenabled == 1)
	{
		// tell hc module to stop transmitting
		indicator = 2;
		while(write(s, &indicator, 1) == 0);
	
		// close connection	
		close(s);	
		printf("Closed connection \n");
	}
	
	// stop motors
	set_motor_speed(STOP, STOP);
	
	// terminate GPIO library
	gpioTerminate();

	printf("Done\n");

	return rc;
}

// initialize the required pins
void GPIO_pin_init(void)
{
	gpioSetMode(MOTOR_LA , PI_OUTPUT);
	gpioSetMode(MOTOR_RA , PI_OUTPUT);
	gpioSetMode(MOTOR_LB , PI_OUTPUT);
	gpioSetMode(MOTOR_RB , PI_OUTPUT);

	gpioSetPWMfrequency(MOTOR_LA , FREQ );
	gpioSetPWMfrequency(MOTOR_LB , FREQ );
	gpioSetPWMfrequency(MOTOR_RA , FREQ );
	gpioSetPWMfrequency(MOTOR_RB , FREQ );
	
}

int set_motor_speed(float left_duty, float right_duty)
{
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
	
	// set PWM, return -1 if unsuccessful
	//if(gpioPWM(MOTOR_LA, la_pwm) != 0)	return -1;
	//if(gpioPWM(MOTOR_RA, ra_pwm) != 0)	return -1;
	//if(gpioPWM(MOTOR_RB, rb_pwm) != 0)	return -1;
	//if(gpioPWM(MOTOR_LB, lb_pwm) != 0)	return -1;
	
	// slightly faster version of above, with no error handling
	gpioPWM(MOTOR_LA, la_pwm);
	gpioPWM(MOTOR_LB, lb_pwm);
	gpioPWM(MOTOR_RA, ra_pwm);
	gpioPWM(MOTOR_RB, rb_pwm);
	
	return 0;
}
