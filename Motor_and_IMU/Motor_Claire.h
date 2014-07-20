#ifndef Motor_Claire_h
#define Motor_Claire_h

#include "Arduino.h"

class Motor_Claire
{
	public:
		Motor_Claire(int pwmPort, int aPort, int bPort);
		void sendPWMCommand(int command);
		void setup();

	private:
		void moveMotor(int speed, int direction);
		int _pwmPort, _aPort, _bPort;
		int _speed, _direction;

		// directions
		static const int FORWARD = 0;
		static const int BACKWARD = 1;
};

#endif