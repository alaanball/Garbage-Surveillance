#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <signal.h>

// raspberry pi program to test working of bluetooth and joystick

static volatile int running = 1;

void sighandler(int dummy)
{
	running = 0;
}

int main(int argc, char **argv)
{
	struct sockaddr_rc addr = { 0 };
	short command[2] = { 0 }, x, y;
	int s, bytes_read;
	char dest[18] = "20:15:04:24:01:30";
	char msg[20]= {0};

	// set as 1 and send to hc module to start transmission;
	// set as 2 and send to hc module to stop transmission
	char indicator;

	signal(SIGINT, sighandler);

	// allocate socket

	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// set the connection parameters (who to connect to)

	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba( dest, &addr.rc_bdaddr );

	// connect to server - bluetooth module
	if(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
	{
		perror("Failed to connect");
		return -1;
	}

	printf("Connected successfully. Waiting for commands \n");

	// enable transmission in case it is disabled
	indicator = 1;
	while(write(s, &indicator, 1) == 0);

	// read data from the arduino module
	while(running)
	{
		bytes_read = read(s, command, sizeof(command));
                if( bytes_read > 0 )
                {
                        x = command[0];
                        y = command[1];

                        printf("x: %d y: %d \n", x, y);
                }

		sleep(1);
	}

	// tell hc module to stop transmitting
	indicator = 2;
	while(write(s, &indicator, 1) == 0);

	// close connection
	close(s);

	printf("Closed connection \n");

	return 0;

}
