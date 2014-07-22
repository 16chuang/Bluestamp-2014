#ifndef PIDController_Claire_h
#define PIDController_Claire_h

#include "Arduino.h"
#include <QueueList.h>

class PIDController_Claire
{
	public:
		PIDController_Claire(float kP, float kI, float kD, Print &print, bool integralQueue=false, bool pGainScheduling=false); 
		// default: custom P and I = false
		
		float compute(float currentReading, float setpoint);
		float feedforwardCompute(float currentReading, float setpoint, float kFF);

		float getErrorSum();

	private:
		float pTerm(float currentReading);
		float iTerm();
		float dTerm();

		float _kP, _kI, _kD;
		bool _integralQueue, _pGainScheduling;
		float _error, _errorSum, _prevError;
		QueueList <float> _errorQueue;
		float _feedforward, _feedback;

		static const float kP_MAX = 70.0;
		static const int MAX_QUEUE_SIZE = 100;

		Print* _printer; // for Serial.println debugging
};

#endif
