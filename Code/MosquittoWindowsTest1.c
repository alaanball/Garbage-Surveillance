// build:  gcc .\MosquittoWindowsTest1.c -o winMosqTest.exe -L"F:\College\Garbage Surveillance - Final Year Project\Code" -lmosquitto

#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>
// ##include <unistd.h>
#include "mosquitto.h"

#define mqtt_host "test.mosquitto.org"
#define mqtt_port 1883

static int run = 1;
int counter = 0;
static volatile char sent = 0;

void sighandler(int dummy)
{
	run = 0;
}

void pub_callback(struct mosquitto *mosq, void *obj, int result)
{
	sent = 1;
}

int main(int argc, char *argv[])
{
	uint8_t reconnect = true;
	char clientid[24];
	char msg[5] = "test", c;
	struct mosquitto *mosq;
	int rc = 0;

	// ctr + c handler
	signal(SIGINT, sighandler);
	
	printf("Starting \n");
	
	mosquitto_lib_init();

	memset(clientid, 0, 24);
	snprintf(clientid, 23, "WindowsTrans");
	mosq = mosquitto_new(clientid, true, 0);

	if(mosq){
		//mosquitto_connect_callback_set(mosq, connect_callback);
		//mosquitto_message_callback_set(mosq, message_callback);
		mosquitto_publish_callback_set(mosq, pub_callback);

	    rc = mosquitto_connect(mosq, mqtt_host, mqtt_port, 60);

		mosquitto_subscribe(mosq, NULL, "GSRtest/test1", 0);

		rc = mosquitto_loop(mosq, -1, 1);
		while(run)
		{
			
			if(run && rc)
			{
				printf("connection error!\n");
				Sleep(10000);
				mosquitto_reconnect(mosq);
			}
			else if(run != 0 && rc == 0)
			{
				printf("Waiting for command (char): ");
				msg[0] = getch();
				msg[1] = '\0';
				printf("%s \n", msg);
				
				mosquitto_publish(mosq, NULL, "GSRtest/test1", 2, msg, 2, false);
				
				while(sent != 1)
				{
					Sleep(500);
					rc = mosquitto_loop(mosq, -1, 1);
				}
				sent = 0;
			}
			Sleep(50);
		}
		mosquitto_destroy(mosq);
	}

	mosquitto_lib_cleanup();
	printf("Done\n");

	return rc;
}