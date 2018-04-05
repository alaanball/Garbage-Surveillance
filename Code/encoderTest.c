#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <signal.h>
#include <pigpio.h>

// These are the GPIO numbers, not the pin numbers - see the rpi schematic for the mapping between the two
#define MOTOR_LA 5	  // pin 29 - left side, 6th from bottom	
#define MOTOR_RA 19   // pin 35 - left side, 3rd from bottom
#define MOTOR_LB 6    // pin 31 - left side, 5th from bottom
#define MOTOR_RB 26   // pin 37 - left side, 2nd from bottom

#define FREQ 5000
#define RANGE 255

#define FORWARD 0.5
#define BACKWARD -0.5
#define STOP 0

// GPIO pins of encoders
#define REAR_LEFT_ENCODER 23
#define REAR_RIGHT_ENCODER 24

// number of encoder GPIO pins
#define ENCODER_PINS 2

// delay (in milliseconds) between intervals at which velocity is calculated
#define VEL_STEP 100

// number of slots in encoder disc
#define SLOTS 20

void GPIO_pin_init(void);
void RLencoderHandler(void);
void RRencoderHandler(void);
void sighandler(int dummy);
void calcVelocity(void);
int set_motor_speed(float left_duty, float right_duty); 	// argument range: -1 to 1, -1 = max speed, reverse; 1 = max speed, forward


// main program status
// set running as 0 to stop the program
volatile int running = 1;

// array of encoder pin numbers and interrupt handlers
int encoderPins[ENCODER_PINS] = {REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
void* encoderHandler[ENCODER_PINS] = {RLencoderHandler, RRencoderHandler};

volatile int RLEcount = 0; // rear left encoder counter
volatile int RREcount = 0; // rear right encoder counter

// RPMs or right and left motors
volatile int rightVel = 0;
volatile int leftVel = 0;

int rightTargetRPM = 300;
int leftTargetRPM = 300;

int error = 5;

int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;

int main(void)
{		 
	float left = FORWARD;
	float right = FORWARD;
	float correc = 0.01;
	int printCounter = 0, leftAvg = 0, rightAvg = 0;
	
	// initialize GPIO library
	if (gpioInitialise() < 0)
	{
		printf("Failed to initialize GPIO. Terminating program \n");
		return -1;
	}
	GPIO_pin_init();
	
	// ctr + c handler
	signal(SIGINT, sighandler);
	
	// initialize timer 0 to callback every VEL_STEP milliseconds
	if(gpioSetTimerFunc(0, VEL_STEP, calcVelocity) != 0)
	{
		printf("Failed  to initialize timer \n");
		return -1;
	}
	
	set_motor_speed(FORWARD, FORWARD);
	
	// display velocity every second
	while(running)
	{
		if(printCounter == 4)
		{
			printf("right velocity (RPM): %d \n", rightAvg);
			printf("left velocity (RPM): %d \n", leftAvg);
			printCounter = 0;
		}
		
		// rightAvg and leftAvg are not reset to 0 because multiplying them
		// by printCounter results in 0 anyway
		rightAvg = (rightAvg * printCounter + rightVel)/(printCounter + 1);
		leftAvg = (leftAvg * printCounter + leftVel)/(printCounter + 1);
		
		if((rightAvg - rightTargetRPM) > error)
		{
			right = right - correc;
			if(right < -1)
				right = -1;
		}
		else if((-rightAvg + rightTargetRPM) > error)
		{
			right = right + correc;
			if(right > 1)
				right = 1;
		}
		
		if((leftAvg - leftTargetRPM) > error)
		{
			left = left - correc;
			if(left < -1)
				left = -1;
		}
		else if((-leftAvg + leftTargetRPM) > error)
		{
			left = left + correc;
			if(left > 1)
				left = 1;
		}
	
		set_motor_speed(left, right);
		
		// delay of x seconds
		sleep(VEL_STEP / 1000);
		printCounter++;
		
	}
	
	// terminate GPIO library
	gpioTerminate();
}

// initialize GPIO pins
void GPIO_pin_init(void)
{
	for(int i = 0; i < ENCODER_PINS; i++)
	{
		gpioSetMode(encoderPins[i], PI_INPUT);
		gpioSetAlertFunc(encoderPins[i], encoderHandler[i]);
	}	
	
	gpioSetMode(MOTOR_LA , PI_OUTPUT);
	gpioSetMode(MOTOR_RA , PI_OUTPUT);
	gpioSetMode(MOTOR_LB , PI_OUTPUT);
	gpioSetMode(MOTOR_RB , PI_OUTPUT);
	
	gpioSetPWMfrequency(MOTOR_LA , FREQ );
	gpioSetPWMfrequency(MOTOR_LB , FREQ );
	gpioSetPWMfrequency(MOTOR_RA , FREQ );
	gpioSetPWMfrequency(MOTOR_RB , FREQ );
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

// callback function which calculates velocity every VEL_STEP milliseconds
void calcVelocity(void)
{
	rightVel = (((float) RREcount) / (2 * SLOTS)) * (60 * 1000 / VEL_STEP); 
	leftVel = (((float) RLEcount) / (2 * SLOTS)) * (60 * 1000 / VEL_STEP);
	RLEcount = 0;
	RREcount = 0;
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
	
	// slightly faster version of above, with no error handling
	gpioPWM(MOTOR_LA, la_pwm);
	gpioPWM(MOTOR_LB, lb_pwm);
	gpioPWM(MOTOR_RA, ra_pwm);
	gpioPWM(MOTOR_RB, rb_pwm);
	
	return 0;
}
