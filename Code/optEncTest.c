// gcc -o optEncTest optEncTest.c -lpigpio -pthread

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <signal.h>
#include <pigpio.h>

// These are the GPIO numbers, not the pin numbers - see the rpi schematic for the mapping between the two
// motor driver outputs
#define MOTOR_LA 5	  // pin 29 - left side, 6th from bottom
#define MOTOR_LB 6    // pin 31 - left side, 5th from bottom
	
#define MOTOR_RA 19   // pin 35 - left side, 3rd from bottom
#define MOTOR_RB 26   // pin 37 - left side, 2nd from bottom

// GPIO pins of encoders
#define REAR_LEFT_ENCODER 23  	//pin 16 - right, 8th from top
#define REAR_RIGHT_ENCODER 24 	//pin 18 - right, 9th from top

#define FREQ 5000
#define RANGE 255

#define FORWARD 0.5
#define BACKWARD -0.5
#define STOP 0

// number of encoder GPIO pins
#define ENCODER_PINS 2

// delay (in milliseconds) between intervals at which velocity is calculated
#define VEL_STEP 100

// number of slots in encoder disc
#define RSLOTS 15
#define LSLOTS 18

void GPIO_pin_init(void);
void RLencoderHandler(void);
void RRencoderHandler(void);
void sighandler(int dummy);
void calcVelocity(void);
int set_motor_speed(float left_duty, float right_duty); 	// argument range: -1 to 1, -1 = max speed, reverse; 1 = max speed, forward
void correctVelocity(void);
void printTimer(void);
void propControl(void);
void PIDControl(void);
void PIDcalcVelocity(void);

// main program status
// set running as 0 to stop the program
volatile int running = 1;

// array of encoder pin numbers and interrupt handlers
int encoderPins[ENCODER_PINS] = {REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
void* encoderHandler[ENCODER_PINS] = {RLencoderHandler, RRencoderHandler};

volatile int RLEcount = 0, LcountTotal = 0; // rear left encoder counter
volatile int RREcount = 0, RcountTotal = 0; // rear right encoder counter

// RPMs of right and left motors
volatile float rightVel = 0;
volatile float leftVel = 0;

int rightTargetRPM = 200;
int leftTargetRPM = 200;

int error = 10;

int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;
float leftAvg = 0, rightAvg = 0;

unsigned long int counter = 0;

float left = STOP;
float right = STOP;
float correc = 0.001;

float kp = 0.0004, ki = 0.00007, kd = 0.001;
float RintegralTerm = 0, LintegralTerm = 0;
float RdervTerm = 0, LdervTerm = 0;
float Rerror = 0, prevRerror = 0;
float Lerror = 0, prevLerror = 0;


int main(void)
{		 
	// initialize GPIO library
	if (gpioInitialise() < 0)
	{
		printf("Failed to initialize GPIO. Terminating program \n");
		return -1;
	}
	GPIO_pin_init();
	
	// ctr + c handler
	signal(SIGINT, sighandler);
	
	// initialize timer 2. Prints RPMs every 2 seconds
	if(gpioSetTimerFunc(1, VEL_STEP, calcVelocity) != 0)
	{
		printf("Failed  to initialize timer 2\n");
		return -1;
	}
	
	// initialize timer 2. Prints RPMs every 2 seconds
	if(gpioSetTimerFunc(2, 1000, printTimer) != 0)
	{
		printf("Failed  to initialize timer 2\n");
		return -1;
	}
	
	set_motor_speed(STOP, STOP);
	sleep(5);
	
	// display velocity every second
	while(running)
	{		

		right = 0; left = 0;
		
		for(int i = 0; ; right < 1 && left < 1)
		{
			right += 0.1;
			left += 0.1;
			printf("%f \n", right);
			
			set_motor_speed(left, right);
			sleep(10);
		}
		// delay of x seconds
		sleep(2);				
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
		gpioGlitchFilter(encoderPins[i], 3000); 
		
		// 400 rpm * (1/60) minutes per second * 40 interrupts per rotation 
		// = 270 interrupts per second at 400 rpm = 1 interrupt every 3.75ms
		// state must persist for at least 3 milliseconds to recognize the state change
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
	LcountTotal++;
}

// Rear Right Encoder interrupt handler
void RRencoderHandler(void)
{
	RREcount++;
	RcountTotal++;
}

//signal handler for ctrl + c (SIGINT)
void sighandler(int dummy)
{
	running = 0;
}

// callback function which calculates velocity every VEL_STEP milliseconds
void calcVelocity(void)
{
	rightAvg = (rightAvg * counter + rightVel)/(counter + 1);
	leftAvg = (leftAvg * counter + leftVel)/(counter + 1);
		
	rightVel = (((float) RREcount) / (2 * RSLOTS)) * (60 * 1000 / VEL_STEP); 
	leftVel = (((float) RLEcount) / (2 * LSLOTS)) * (60 * 1000 / VEL_STEP);
	RLEcount = 0;
	RREcount = 0;
	
	counter++;
	if(counter == 5)
		counter = 0;
}

void PIDcalcVelocity(void)
{
	prevRerror = rightAvg - rightTargetRPM;
	prevLerror = leftAvg - leftTargetRPM;
	
	rightVel = (((float) RREcount) / (2 * SLOTS)) * (60 * 1000 / VEL_STEP); 
	leftVel = (((float) RLEcount) / (2 * SLOTS)) * (60 * 1000 / VEL_STEP);
	
	RLEcount = 0;
	RREcount = 0;
	
	rightAvg = (rightAvg * counter + rightVel)/(counter + 1);
	leftAvg = (leftAvg * counter + leftVel)/(counter + 1);
	
	Rerror = rightAvg - rightTargetRPM;
	Lerror = leftAvg - leftTargetRPM;
	
	RintegralTerm += ki * Rerror;
	LintegralTerm += ki * Lerror;
	
	RdervTerm = kd * (Rerror - prevRerror);
	LdervTerm = kd * (Lerror - prevLerror);
	
	counter++;
	if(counter == 5)
		counter = 0;	
}

void printTimer(void)
{
	printf("right counter: %d %f \n", RcountTotal, rightVel);
	printf("left counter: %d %f\n", LcountTotal, leftVel);		
}

void correctVelocity(void)
{	
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
}


void propControl(void)
{
	if((rightAvg - rightTargetRPM) * (rightAvg - rightTargetRPM)  > error)
	{
		right = right + kp * (-rightAvg + rightTargetRPM);
		if(right < -1)
			right = -1;
		if(right > 1)
			right = 1;
	}
		
	if((leftAvg - leftTargetRPM) * (leftAvg - leftTargetRPM) > error)
	{
		left = left + kp * (-leftAvg + leftTargetRPM);
		if(left < -1)
			left = -1;
		if(left > 1)
			left = 1;
	}
	
	set_motor_speed(left, right);	
}

void PIDControl(void)
{	
		right = kp * Rerror + RintegralTerm + RdervTerm;
		if(right < -1)
			right = -1;
		if(right > 1)
			right = 1;
	

		left = kp * Lerror + LintegralTerm + LdervTerm;
		if(left < -1)
			left = -1;
		if(left > 1)
			left = 1;
	
	
	set_motor_speed(left, right);	
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
