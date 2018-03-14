// Deprecated motor control code - not tested, not using it currently
// Joystick control is implemented as follows
// 
//            |	   |
//			  | up |
//       -----------------
//   left     | st-|    right
//            | op |
//       -----------------
//            | do-|
//            | wn |

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <pigpio.h>

#define MOTOR_LA 5		// Note that these are the GPIO numbers, not the pin numbers - see the pigpio website for the map between the two
#define MOTOR_RA 6
#define MOTOR_LB 19
#define MOTOR_RB 26
#define EN1 20
#define EN2 21
#define FREQ 5000

#define XRANGE 1024
#define YRANGE 1024

#define YTOPTHRESH 720
#define YBOTTHRESH 280

#define XLEFTTHRESH 280
#define XRIGHTTHRESH 720

#define RANGE 255

#define FORWARD 0.5
#define BACKWARD -0.5
#define STOP 0

int setPWM( int left, int right );
void GPIO_pin_init(void);					// sets GPIO pin modes, frequencies, and states
int set_motor_speed(float left_duty, float right_duty); // argument range: -1 to 1
int ra_pwm, la_pwm, rb_pwm, lb_pwm = 0;

int main(int argc, char **argv)
{
	struct sockaddr_rc addr = { 0 };
	int command[3] = { 0 };
	int s, bytes_read; 
	int opt = sizeof(addr);
	char dest[18] = "01:23:45:67:89:AB";
	
	float duty_cycle = 0.75;
	int FORWARD = (int) (RANGE * duty_cycle); 
	int BACKWARD = (int) (RANGE * (duty_cycle - 0.5));
	int STOP = (int) (RANGE *  0.5);
	
	int joystick_x = 0, joystick_y = 0, joystick_sw = 0;
	
	// initialise GPIO library
	
	if (gpioInitialise() < 0)
	{
		printf("Failed to initialize GPIO. Terminating program \n");
		return -1;
	}
	
	GPIO_pin_init();
	
	// allocate socket
	
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	
	// set the connection parameters (who to connect to)
	
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba( dest, &addr.rc_bdaddr );
	
	// connect to server - arduino's bluetooth module
	
	if(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
	{
		perror("Failed to connect");
		return -1;
	}
	
	// read data from the arduino module
	
	while(1)
	{
		bytes_read = recv(s, command, sizeof(command), MSG_PEEK);
		if( bytes_read > 0 ) 
		{
			// motor control based on joystick values
			
			joystick_sw = command[0];
			joystick_x = command[1];
			joystick_y = command[2];
			
			if(joystick_sw == 1)
			{
					// stop motors and exit program
					
					set_motor_speed(STOP, STOP);
					
					break;
			}
			
			if((joystick_x >= XRIGHTTHRESH) && (joystick_y >= YTOPTHRESH))
			{
				// top right
				
				set_motor_speed(FORWARD, STOP);
				
			}
			else if((joystick_x <= XLEFTTHRESH) && (joystick_y >= YTOPTHRESH))
			{
				// top left
				
				set_motor_speed(STOP, FORWARD);
				
			}
			else if((joystick_x <= XLEFTTHRESH) && (joystick_y <= YBOTTHRESH))
			{
				// bottom left
				
				set_motor_speed(STOP, BACKWARD);
				
			}
			else if((joystick_x <= XRIGHTTHRESH) && (joystick_y >= YBOTTHRESH))
			{
				// bottom right
				
				set_motor_speed(BACKWARD, STOP);
				
			}
			else if((joystick_x <= XRIGHTTHRESH) && (joystick_x >= XLEFTTHRESH) && (joystick_y >= YTOPTHRESH))
			{
				// up
				
				set_motor_speed(FORWARD, FORWARD);
				
			}
			else if((joystick_x <= XRIGHTTHRESH) && (joystick_x >= XLEFTTHRESH) && (joystick_y <= YBOTTHRESH))
			{
				// down
				
				set_motor_speed(BACKWARD, BACKWARD);
				
			}
			else if((joystick_x >= XRIGHTTHRESH) && (joystick_y >= YBOTTHRESH) && (joystick_y <= YTOPTHRESH))
			{
				// right
				
				set_motor_speed(FORWARD, BACKWARD);
				
			}
			else if((joystick_x <= XLEFTTHRESH) && (joystick_y >= YBOTTHRESH) && (joystick_y <= YTOPTHRESH))
			{
				// left
				
				set_motor_speed(BACKWARD, FORWARD);
				
			}
			else
			{
				// stay still
				
				set_motor_speed(STOP, STOP);
				
			}
		}
	}
	
	// close connection
	
	close(s);
	
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
	
	gpioSetMode(EN1 , PI_INPUT);
	gpioSetMode(EN2 , PI_INPUT);
	
	gpioSetPWMfrequency(MOTOR_LA , FREQ );
	gpioSetPWMfrequency(MOTOR_RA , FREQ );
	gpioSetPWMfrequency(MOTOR_LB , FREQ );
	gpioSetPWMfrequency(MOTOR_RB , FREQ );
	
}

/*int set_motor_speed(int left_speed, int right_speed)
{
	if(gpioPWM(MOTOR_LA, left_speed) != 0)	return -1;
	if(gpioPWM(MOTOR_RA, right_speed) != 0)	return -1;
	
	return 0;
}*/

int set_motor_speed(float left_duty, float right_duty)
{
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