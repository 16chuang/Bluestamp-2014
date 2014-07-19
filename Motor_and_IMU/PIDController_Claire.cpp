#include "Arduino.h"
#include <QueueList.h>
#include "PIDController_Claire.h"

PIDController_Claire::PIDController_Claire(float kP, float kI, float kD, Print &print, bool integralQueue, bool pGainScheduling)
{
	// gains
	_kP = kP;
	_kI = kI;
	_kD = kD;

	// custom P and I;
	_integralQueue = integralQueue;
	_pGainScheduling = pGainScheduling;

	// private compute variables
	_error = 0;
	_errorSum = 0;
	_prevError = 0;

	_printer = &print;
}

float PIDController_Claire::compute(float currentReading, float setpoint) 
{
	_error = currentReading - setpoint;
	return (pTerm(currentReading) + iTerm() + dTerm());
}

float PIDController_Claire::pTerm(float currentReading) 
{
	if (_pGainScheduling) {
		// special function for pitch to PWM
		_kP = min(abs(((55 * pow(currentReading, 2)) + (20 * currentReading) + 10)), kP_MAX);
	}

	return (_kP * _error);
}

float PIDController_Claire::iTerm() 
{
	if (_integralQueue) {
		// update queue and sum with current error
		_errorQueue.push(_error);
		_errorSum += _error;

		if (_errorQueue.count() > MAX_QUEUE_SIZE) { // keep most recent __ # of values
			_errorSum -= _errorQueue.pop();
		}

	} else {
		_errorSum += _error;
		// _printer->println(_errorSum);
	}

	return (_kI * _errorSum);
}

float PIDController_Claire::dTerm() 
{
	float dTerm = _kD * (_error - _prevError);
	_prevError = _error;
	return dTerm;
}
