#include "paho.mqtt.c/src/MQTTClient.h"
#include <stdio.h>
#include <string.h>

int main(){

	MQTTClient client;
	MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
	MQTTClient_message pubsg = MQTTClient_message_initializer;
	int rc;
	int topicLen;

	int createdClient = MQTTClient_create(&client, "tcp://localhost:1833", "ReceiverClient", MQTTCLIENT_PERSISTENCE_NONE, NULL);
	conn_opts.keepAliveInterval = 20;
	conn_opts.cleansession = 1;

	const char* temp = "ControllerData";
	char* topic;
	topic = strcpy(topic, temp);
	MQTTClient_message* data;

	while(1){

		int receive = MQTTClient_receive(client, &topic, &topicLen, &data, 60);
		printf("s", data->payload);

	}

}
