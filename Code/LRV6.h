
#define mqtt_host "test.mosquitto.org"
#define mqtt_port 1883

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
#define RANGE 10000

#define XM 1024
#define YM 1024
#define XC 512
#define YC 512
#define RAD 150
#define YTOPTHRESH 720
#define YBOTTHRESH 280

#define XLEFTTHRESH 280
#define XRIGHTTHRESH 720

#define FORWARD 0.5
#define BACKWARD -0.5
#define STOP 0

// possible values of learnStatus and repeatStatus
#define INACTIVE 0
#define ACTIVE 1
#define PAUSED 2
#define DONE 3

// number of GPIO pins
#define ENCODER_PINS 2
#define DRIVER_PINS 4

// delay (in milliseconds) between intervals at which velocity is calculated
#define VEL_STEP 50
#define AVG_COUNTER 10
#define ACTIVE_RPM 80

// number of slots in encoder disc
#define RSLOTS 20
#define LSLOTS 11

// maximum number of minutes the program can be in LEARN mode
#define MAX_MINUTES 10

// time interval in milliseconds between times at which commands are stored
#define CMD_TIME_RESOLUTION 100

#define CALC_SIZE 10 * 60 * 1000 / 100
#define DIST 17.5
#define RADIUS 3.6

#define PI 3.14159265

// function headers
void GPIO_pin_init(void);
void RLencoderHandler(void);
void RRencoderHandler(void);
void sighandler(int dummy);
void calcVelocity(void);
int set_motor_speed(double left_duty, double right_duty); 	// argument range: -1 to 1, -1 = max speed, reverse; 1 = max speed, forward
void correctVelocity(void);
void printTimer(void);
void propControl(void);
void PIDControl(void);
void PIDcalcVelocity(void);
void connect_callback(struct mosquitto *mosq, void *obj, int result);
void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);
void set_forward(void);
void set_reverse(void);
void set_right(void);
void set_left(void);
void stop_motors(void);
void learnTimer(void);
void repeatTimer(void);
void switchTick(void);
void initLearnMode(void);
void initRepeatMode(void);
void pauseLearnMode(void);
void pauseRepeatMode(void);
void resumeLearnMode(void);
void resumeRepeatMode(void);
void stopLearnMode(void);
void stopRepeatMode(void);
void speedMatch(void);
void BTdisc(void);
