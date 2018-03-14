#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <signal.h>
#include <pigpio.h>

// These are the GPIO numbers, not the pin numbers - see the rpi schematic for the mapping between the two
#define MOTOR_LA 5		
#define MOTOR_RA 6
#define MOTOR_LB 19
#define MOTOR_RB 26
#define EN1 20
#define EN2 21
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

static volatile int running = 1;							// set as 0 when ctrl + c is pressed; program then exits main while loop

void GPIO_pin_init(void);									// sets GPIO pin modes, frequencies, and states
int set_motor_speed(float left_duty, float right_duty); 	// argument range: -1 to 1, -1 = max speed, reverse; 1 = max speed, forward
int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;

//signal handler for ctrl + c (SIGINT)
void sighandler(int dummy)
{
	running = 0;
}

int main(int argc, char **argv)
{
	struct sockaddr_rc addr = { 0 };
	short command[2] = { 0 };
	int s, bytes_read, x, y; 
	char dest[18] = "20:15:04:24:01:30";
	int motorPins[4] = {MOTOR_LA, MOTOR_LB, MOTOR_RA, MOTOR_RB};
	
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
		return -1;
	}
	
	printf("Connected successfully. Waiting for commands \n");
	
	// enable transmission from Bluetooth module in case it is disabled
	indicator = 1;
	while(write(s, &indicator, 1) == 0);
	
	// read data from the hc module and take action
	while(running)
	{	
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
				//printf("stop \n");
			}
			else if( y - x >= 0 )
			{
				if(y + x >= YM)
				{
					// up
					set_motor_speed(FORWARD, FORWARD);
					//printf("up \n");
				}
				else
				{
					// left
					set_motor_speed(BACKWARD, FORWARD);
					//printf("left \n");
				}
			}
			else
			{
				if(y + x >= YM )
				{
					// right
					set_motor_speed(FORWARD, BACKWARD);
					//printf("right \n");
				}
				else
				{
					// down
					set_motor_speed(BACKWARD, BACKWARD);
					//printf("down \n");
				}
			}
		}		
		sleep(1);
	}
	
	// stop motors
	set_motor_speed(STOP, STOP);
	
	// tell hc module to stop transmitting
	indicator = 2;
	while(write(s, &indicator, 1) == 0);
	
	// close connection	
	close(s);	
	printf("Closed connection \n");
	
	// terminate GPIO library
	gpioTerminate();
	
	return 0;
	
}

// initialize the required pins
void GPIO_pin_init(void)
{
	gpioSetMode(MOTOR_LA , PI_OUTPUT);
	gpioSetMode(MOTOR_RA , PI_OUTPUT);
	gpioSetMode(MOTOR_LB , PI_OUTPUT);
	gpioSetMode(MOTOR_RB , PI_OUTPUT);
	
	gpioSetMode(EN1 , PI_OUTPUT);
	gpioSetMode(EN2 , PI_OUTPUT);
	
	gpioWrite(EN1, 1);
	gpioWrite(EN2, 1);

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
