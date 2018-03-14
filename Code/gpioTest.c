#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>

// These are the GPIO numbers, not the pin numbers - see the raspberry pi schematic for the mapping between the two
#define MOTOR_LA 5		
#define MOTOR_RA 6
#define MOTOR_LB 19
#define MOTOR_RB 26
#define EN1 20
#define EN2 21
#define FREQ 5000
#define RANGE 255

void GPIO_pin_init(void);					// sets GPIO pin modes, frequencies, and states
int set_motor_speed(float left_duty, float right_duty); // arguments are between -1 and 1

int main(int argc, char **argv)
{	
	// initialize GPIO library
	
	if (gpioInitialise() < 0)
	{
		printf("Failed to initialize GPIO. Terminating program \n");
		return -1;
	}
	
	GPIO_pin_init();
	
	if(set_motor_speed(1, 0.5) != 0)
		printf("Unable to set speed");
		
	sleep(1000);
	
	// terminate GPIO library
	
	gpioTerminate();
	
	return 0;
	
}


void GPIO_pin_init(void)
{
	gpioSetMode(MOTOR_LA , PI_INPUT);
	gpioSetMode(MOTOR_RA , PI_INPUT);
	gpioSetMode(MOTOR_LB , PI_INPUT);
	gpioSetMode(MOTOR_RB , PI_INPUT);
	
	gpioSetMode(EN1 , PI_OUTPUT);
	gpioSetMode(EN2 , PI_OUTPUT);
	
	gpioWrite(EN1, 1);
	gpioWrite(EN2, 1);
	
	gpioSetPWMfrequency(MOTOR_LA , FREQ );
	gpioSetPWMfrequency(MOTOR_RA , FREQ );
	gpioSetPWMfrequency(MOTOR_LB , FREQ );
	gpioSetPWMfrequency(MOTOR_RB , FREQ );	
}

int set_motor_speed(float left_duty, float right_duty)
{
	int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;

	left_duty = 0.5 + left_duty / 2;
	right_duty = 0.5 + right_duty / 2;
	
	la_pwm = (int) (RANGE * left_duty);
	lb_pwm = (int) (RANGE * (1 - left_duty));
	ra_pwm = (int) (RANGE * right_duty);
	rb_pwm = (int) (RANGE * (1 - right_duty));
	
	if(gpioPWM(MOTOR_LA, la_pwm) != 0)	return -1;
	if(gpioPWM(MOTOR_RA, ra_pwm) != 0)	return -1;
	if(gpioPWM(MOTOR_RB, rb_pwm) != 0)	return -1;
	if(gpioPWM(MOTOR_LB, lb_pwm) != 0)	return -1;
	
	return 0;
}
