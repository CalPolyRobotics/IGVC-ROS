#include "SerialComms.hpp"
#include "CaddyBoardControl.hpp"

#include <iostream>
#include "unistd.h"

COMM_STATUS CaddyBoardControl::initializeComms(int portNumber, int baud){
	atmega.initializeComms(portNumber,baud);
}

int CaddyBoardControl::setFNR(char direction){

	int count = 0;
	Packet FNRPacket = {4,0,1,0};
	FNRPacket.payload = (unsigned char*)&direction;
		
	while(atmega.sendPacket(&FNRPacket) != COMM_SUCCESS){
		std::cout << "Failure sending packet. Trying again\n";
		usleep(400000);
		count++;
		if(count > 5){
			break;
		}
	}
	if(count > 5){
		std::cout << "Command unsuccessful!\n";
		return -1;
	} else {
		std::cout << "Successfully sent command\n";
		return 0;
	}
}
