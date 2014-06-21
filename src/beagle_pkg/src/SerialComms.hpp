#ifndef __READ_PORT_H__
#define __READ_PORT_H__



typedef struct {
	unsigned char groupID;
	unsigned char cmd;
	unsigned char payloadSize;
	unsigned char *payload;
} Packet;

typedef struct {
	int size;
	unsigned char *payload;
} ReturnPayload;

enum COMM_STATUS {COMM_SUCCESS,COMM_FAILURE,COMM_TIMEOUT,COMM_BAD_CHECKSUM,COMM_NACK};

class SerialComms {

	public:
	SerialComms();

	/* Initializes	the portNumber and baud rate for
	 * communication, as well as making the actual
	 * initialization call to the library.
	 */
	COMM_STATUS initializeComms(int portNumber, int baud);

	/* Send a single command packet and return the
	 * result of trying to send the packet. The actual
	 * resulting payload can be recieved by calling 
	 * getResults.
	 */
	COMM_STATUS sendPacket(Packet *packet);

	/* Get the payload bytes from a recent serial
	 * transaction. Will be overwritten after the
	 * next call to sendPacket.
	 */
	ReturnPayload getResults();

	private:
	
	int portNumber;
	int baud;	
	Packet *packet;
	ReturnPayload returnPayload;
	int status;
	const static unsigned char ACK_BYTE = 128;
	const static unsigned char NACK_BYTE = 0;
	const static int timeoutLen = 200;
	const static int maxNumTries = 3;

	/* Sends the 4 byte command header. Waits for an
	 * acknowledgement byte and will repeat the process
	 * up to maxNumTries times.
	 */
	COMM_STATUS sendHeader();

	/* Sends the payload inside the packet as well as
	 * a checksum. Then waits for an acknowlegment.
	 */
	COMM_STATUS sendPayload();

	/* Waits for an acknowledgement byte. Will timeout
	 * after the timeout counter reaches 200.
	 */
	COMM_STATUS waitForAck();

	/* Sends either an ACK or NACK byte as defined by
	 * either ACK_BYTE or NACK_BYTE.
	 */
	void sendAck();
	void sendNack();

	/* Recives three bytes from the ATmega.
 	 * The first byte is the cmd sent earlier.
 	 * The second byte is the size of the following payload.
 	 * The final byte is a checksum of the two eariler bytes.
 	 * The Beagle Board then respons with an ACK or a NACK.
	 * The process is repeated up to maxNumTries if a bad
	 * checksum is calculated.
 	 */
	COMM_STATUS recievePayloadHeader();


	/* Recieves all the required payload bytes from
	 * the ATmega. Also sends a checksum and waits
	 * for an acknowledgement and repeats the process
	 * up to maxNumTries times.
	 */
	COMM_STATUS recievePayload();
		
};

#endif
