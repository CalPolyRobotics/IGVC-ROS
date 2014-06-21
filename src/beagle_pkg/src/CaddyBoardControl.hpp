#include "SerialComms.hpp"


class CaddyBoardControl {

	public:
	
	COMM_STATUS initializeComms(int portNumber, int baud);

	int setFNR(char direction);
	
	int setFNRForward() {return setFNR(1);}
	int setFNRNeutral() {return setFNR(0);}
	int setFNRReverse() {return setFNR(-1);}


	private:

	SerialComms atmega;

};
