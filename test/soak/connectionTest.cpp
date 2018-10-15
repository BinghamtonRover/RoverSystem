#include "../../src/network/network.hpp"
#include <iostream>
#include <ctime>
#include <string>
#include <cmath>
#include <iomanip>

int main(){
	//Establishing the two network connections
	network::Connection sideA;
	network::Connection sideB;

	//Checking to see if they can both connect to localhost on port 8080
	network::Error error1; 
	network::Error error2;
	if( (error1 = network::connect(&sideA,"127.0.0.1",8081,8080)) != network::Error::OK){
		fprintf(stderr, "Side A failed to connect to localhost!\n");
		return 1;
	}
	if( (error2 = network::connect(&sideB,"127.0.0.1",8080,8081)) != network::Error::OK){
		fprintf(stderr, "Side B failed to connect to localhost!\n");
		return 1;
	}

	std::clock_t start = std::clock();
	double duration;
	int messagesReceived = 0;
	double timeOfLastMessage = 0;
	double averageTime = 0.0;
	while(true) {
		//Code for side A sending messages to side B
		network::poll_incoming(&sideA);
		network::Message messageA;
		while(network::dequeue_incoming(&sideA,&messageA)){
			//Doesn't matter what packets are going to side A since this is a test
			network::return_incoming_buffer(messageA.buffer);
		}
		//Sending a message
		network::Buffer * outgoing = network::get_outgoing_buffer();
		network::queue_outgoing(&sideA,network::MessageType::HEARTBEAT,outgoing);
		network::drain_outgoing(&sideA);

		//Code for side B recieving messages to side A
		network::poll_incoming(&sideB);
		network::Message messageB;
		while(network::dequeue_incoming(&sideB,&messageB)){
			//Check for a heartbeat message
			if(messageB.type == network::MessageType::HEARTBEAT){
				messagesReceived++;
				double currentTime = ((std::clock() - start) / (double) CLOCKS_PER_SEC) * 1000;
				double timeOfMessage = currentTime-timeOfLastMessage;
				std::cout<< "Side B received message #" << messagesReceived << " in " << std::setprecision(3) << timeOfMessage << " milliseconds!\n";
				timeOfLastMessage = currentTime;
				averageTime = (averageTime + currentTime)/messagesReceived;
			}
			network::return_incoming_buffer(messageB.buffer);
		}
		//As of right now, can't figure out how to change it so the delay doesn't mess up when to break from the loop
		if(messagesReceived >= 50)
			break;
		
	}
	duration = (( std::clock() - start) / (double) CLOCKS_PER_SEC) * 1000;
	std::setprecision(0);
	std::cout<<"It took " << std::setprecision(4) << duration <<  " milliseconds to send " << messagesReceived << " messages from side A to side B with an average of " << averageTime << " milliseconds per message.\n";
	system("python3 endSoak.py");	
	return 0;

}
