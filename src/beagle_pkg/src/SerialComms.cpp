#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"
#include "SerialComms.hpp"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

using namespace std;

SerialComms::SerialComms(){
	returnPayload.payload == NULL;
}

COMM_STATUS SerialComms::sendHeader(){

	int tryCount;

	int checksum = packet->groupID;
	checksum += packet->cmd + packet->payloadSize;

	for(tryCount = 0;tryCount < maxNumTries;tryCount++){
   	RS232_SendByte(portNumber, packet->groupID);
   	usleep(1000);
   
		RS232_SendByte(portNumber, packet->cmd);
   	usleep(1000);
   
		RS232_SendByte(portNumber, packet->payloadSize);
   	usleep(1000);
   
		RS232_SendByte(portNumber, checksum);

		switch(waitForAck()){
		case COMM_FAILURE:
			return COMM_FAILURE;	
		case COMM_TIMEOUT:
			return COMM_TIMEOUT;	
		case COMM_NACK:
			break;
		case COMM_SUCCESS:
			return COMM_SUCCESS;
		}
	}

	return COMM_NACK;

}

COMM_STATUS SerialComms::initializeComms(int portNumber, int baud){
	this->portNumber = portNumber;
	this->baud = baud;
	
	if(RS232_OpenComport(portNumber,baud) != 0){
		return COMM_FAILURE;
	} else {
		return COMM_SUCCESS;
	}
}
  
COMM_STATUS SerialComms::waitForAck(){

	unsigned char ack;
	int timeout = timeoutLen;
	
	while(RS232_PollComport(portNumber,&ack,1) == 0){
		if(timeout == 0){
			return COMM_TIMEOUT;
		} else {
			timeout--;
		}
		usleep(500);
	}
	if(ack == ACK_BYTE){
		return COMM_SUCCESS;
	} else if(ack == NACK_BYTE){
		return COMM_NACK;
	} else {
		return COMM_FAILURE;
	}
}	

COMM_STATUS SerialComms::sendPayload(){

	int tryCount;
	int byteCount;
	unsigned char checksum = 0;

	for(tryCount = 0;tryCount < maxNumTries;tryCount++){
		for(byteCount = 0;byteCount < packet->payloadSize; byteCount++){
			RS232_SendByte(portNumber,packet->payload[byteCount]);
			checksum += (unsigned char)packet->payload[byteCount];
			usleep(1000);
		}

		RS232_SendByte(portNumber,checksum);
	
		switch(waitForAck()){
		case COMM_FAILURE:
			return COMM_FAILURE;
		case COMM_TIMEOUT:
			return COMM_TIMEOUT;
		case COMM_NACK:
			break;
		case COMM_SUCCESS:
			return COMM_SUCCESS;
		}
	}
	return COMM_NACK;
}

void SerialComms::sendAck(){
	RS232_SendByte(portNumber,ACK_BYTE);
}

void SerialComms::sendNack(){
	RS232_SendByte(portNumber,NACK_BYTE);
}

COMM_STATUS SerialComms::recievePayloadHeader(){
	unsigned char recievedByte;
	int timeout = timeoutLen;
	int tryCount;

	for(tryCount = 0;tryCount < maxNumTries;tryCount++){	
		while(RS232_PollComport(portNumber,&recievedByte,1) == 0){
			if(timeout){
				timeout--;
			} else {
				return COMM_TIMEOUT;
			}
			usleep(500);
		}
	
		if(recievedByte != packet->cmd){
			printf("Recieved %hhX expected %hhX\n",recievedByte,packet->cmd);
			return COMM_FAILURE;	
		}
	
		timeout = timeoutLen;
		while(RS232_PollComport(portNumber,&recievedByte,1) == 0){
			if(timeout){
				timeout--;
			} else {
				return COMM_TIMEOUT;
			}
			usleep(500);
		}

		returnPayload.size = recievedByte;
      printf("Set payload size to %d\n",recievedByte);
	
		timeout = timeoutLen;
		while(RS232_PollComport(portNumber,&recievedByte,1) == 0){
			if(timeout){
				timeout--;
			} else {
				return COMM_TIMEOUT;
			}
			usleep(500);
		}
		
		if(returnPayload.size + packet->cmd == recievedByte){
			sendAck();
			return COMM_SUCCESS;
		} else {
			sendNack();
		}				
	}
	return COMM_NACK;
}
	
COMM_STATUS SerialComms::recievePayload(){

	int timeout;
	int byteCount;
	int tryCount;
	unsigned char recievedByte;
	unsigned char checksum = 0;

	if(returnPayload.payload != NULL){
		//FIXME: Not having this free WILL cause a memory leak!!!!
		//free(returnPayload.payload);
	}
	returnPayload.payload = (unsigned char*)malloc(returnPayload.size);


	for(tryCount = 0;tryCount < maxNumTries;tryCount++){	
		checksum = 0;
		
		for(byteCount = 0;byteCount < returnPayload.size;byteCount++){
			timeout = timeoutLen;
			while(RS232_PollComport(portNumber,&recievedByte,1) == 0){
				if(timeout){
					timeout--;
				} else {
               printf("Timed out recieving data\n");
					return COMM_TIMEOUT;
				}
				usleep(500);
			}
			returnPayload.payload[byteCount] = recievedByte;
         printf("Recieved byte %d : %hhX\n",byteCount,recievedByte);
			checksum += recievedByte;
		}
	
		timeout = timeoutLen;
		while(RS232_PollComport(portNumber,&recievedByte,1) == 0){
			if(timeout){
				timeout--;
			} else {
            printf("Timed out recieving checksum\n");
				return COMM_TIMEOUT;
			}
			usleep(500);
		}
	
		if(recievedByte == checksum){
         printf("Valid Checksum\n");
			sendAck();
			return COMM_SUCCESS;
		} else {
         printf("Bad Checksum\n");
			sendNack();
		}

	}

}
 
COMM_STATUS SerialComms::sendPacket(Packet *packet){

	char recievedByte;
	int timeout;
	int tryCount;
	int byteCount;	

	this->packet = packet;

	switch(sendHeader()){
	case COMM_FAILURE:
		cerr << "COMM_FAIURE line " << __LINE__ << endl;
		return COMM_FAILURE;
	case COMM_TIMEOUT:
		cerr << "COMM_TIMEOUT line " << __LINE__ << endl;
		return COMM_TIMEOUT;
	case COMM_NACK:
		cerr << "COMM_NACK line " << __LINE__ << endl;
		return COMM_NACK;
	case COMM_SUCCESS:
		break;
	}

	if(packet->payloadSize > 0){
		
		usleep(1000);

		switch(sendPayload()){
		case COMM_FAILURE:
			cout << "COMM_FAIURE line " << __LINE__ << endl;
			return COMM_FAILURE;
		case COMM_TIMEOUT:
			cout << "COMM_TIMEOUT line " << __LINE__ << endl;
			return COMM_TIMEOUT;
		case COMM_NACK:
			cout << "COMM_NACK line " << __LINE__ << endl;
			return COMM_NACK;
		case COMM_SUCCESS:
			break;
		}	
		
	}

	switch(recievePayloadHeader()){
	case COMM_FAILURE:
		cerr << "COMM_FAIURE line " << __LINE__ << endl;
		return COMM_FAILURE;
	case COMM_TIMEOUT:
		cerr << "COMM_TIMEOUT line " << __LINE__ << endl;
		return COMM_TIMEOUT;
	case COMM_BAD_CHECKSUM:
		cerr << "COMM_BAD_CHECKSUM line " << __LINE__ << endl;
		return COMM_BAD_CHECKSUM;
	case COMM_SUCCESS:
		break;
	}

   printf("returnPayload size: %d\n",returnPayload.size);

	if(returnPayload.size != 0){

		switch(recievePayload()){
		case COMM_FAILURE:
			cerr << "COMM_FAIURE line " << __LINE__ << endl;
			return COMM_FAILURE;
		case COMM_TIMEOUT:
			cerr << "COMM_TIMEOUT line " << __LINE__ << endl;
			return COMM_TIMEOUT;
		case COMM_BAD_CHECKSUM:
			cerr << "COMM_BAD_CHECKSUM line " << __LINE__ << endl;
			return COMM_BAD_CHECKSUM;
		case COMM_SUCCESS:
			break;
		}
	}

	return COMM_SUCCESS;
}

ReturnPayload SerialComms::getResults(){
	return returnPayload;
}
	
