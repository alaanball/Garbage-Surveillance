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
char* msg = "test";


void connect_callback(struct mosquitto *mosq, void *obj, int result)
{
	printf("connect callback, rc=%d\n", result);
}

void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
	bool match = 0;
	printf("got message '%.*s' for topic '%s'\n", message->payloadlen, (char*) message->payload, message->topic);
	

	//STOP after receiving 1 message
	run = 0;

	/*mosquitto_topic_matches_sub("test/t1", message->topic, &match);
	if (match) {
		printf("got message for ADC topic\n");
	}*/

}

int main(int argc, char *argv[])
{
	uint8_t reconnect = true;
	char clientid[24];
	
	struct mosquitto *mosq;
	int rc = 0;

	printf("Starting \n");
	

	mosquitto_lib_init();

	memset(clientid, 0, 24);
	snprintf(clientid, 23, "ID14465");
	mosq = mosquitto_new(clientid, true, 0);

	if(mosq){
		mosquitto_connect_callback_set(mosq, connect_callback);
		mosquitto_message_callback_set(mosq, message_callback);

	    rc = mosquitto_connect(mosq, mqtt_host, mqtt_port, 60);

		

		while(run){
			rc = mosquitto_loop(mosq, -1, 1);
			if(run && rc){
				printf("connection error!\n");
				
				sleep(10);
				mosquitto_reconnect(mosq);
			}
			else
			{ run = 0;}
		}
		for(int i = 0; i < 3; i++)
		{
			mosquitto_publish(mosq, NULL, "GSRtest/test1", 4, msg, 2, false);
			sleep(3);
		}
		mosquitto_destroy(mosq);
	}

	mosquitto_lib_cleanup();
	printf("Done\n");

	return rc;
}