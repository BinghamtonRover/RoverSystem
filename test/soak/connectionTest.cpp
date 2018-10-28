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

	//Checking to see if they can both connect to localhost with side A and side B listening to each other on ports 8080 and 8081 respectively
	network::Error error; 
	if( (error = network::connect(&sideA,"127.0.0.1",8081,8080)) != network::Error::OK){
		fprintf(stderr, "Side A failed to connect to localhost!\n");
		return 1;
	}
	if( (error = network::connect(&sideB,"127.0.0.1",8080,8081)) != network::Error::OK){
		fprintf(stderr, "Side B failed to connect to localhost!\n");
		fprintf(stdout,"%d\n",static_cast<int>(error));
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
				//If a message was received, icrement message counter by one and calculate the time stuff as well as print out the information
				messagesReceived++;
				double currentTime = ((std::clock() - start) / (double) CLOCKS_PER_SEC) * 1000;
				double timeOfMessage = currentTime-timeOfLastMessage;
				std::cout<< "Side B received message #" << messagesReceived << " in " << std::setprecision(3) << timeOfMessage << " milliseconds!\n";
				timeOfLastMessage = currentTime;
				averageTime = (averageTime + currentTime)/messagesReceived;
			}
			network::return_incoming_buffer(messageB.buffer);
			//Check to see if we got to 50 messages received (if statement in here in case there are more messages in the dequeue that we don't care about)
			if (messagesReceived >= 50)
				break;
		}
		//Breaking out of the communication loop once side B has 50 messages
		if (messagesReceived >= 50)
			break;
		
	}
	duration = (( std::clock() - start) / (double) CLOCKS_PER_SEC) * 1000;
	std::setprecision(0);
	std::cout<<"It took " << std::setprecision(4) << duration <<  " milliseconds to send " << messagesReceived << " messages from side A to side B with an average of " << averageTime << " milliseconds per message.\n";
	return 0;

}
