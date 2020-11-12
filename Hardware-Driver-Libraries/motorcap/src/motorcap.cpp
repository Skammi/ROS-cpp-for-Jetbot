/*
 *	file :			motorcap.h
 *	Project :		jetbot
 *	Last edit :		23 Oct 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.6	201022	Added variant to runMotor to accommodate differential drive
 *	0.0.5	200930	Changed the control of the motors based on the
 *					Reverse-engineered Waveshare python code.
 *					Changed velocity back to 0--100
 *	0.0.4	200927	Changed velocity from 0--100 to 0--4095
 *	0.0.3	200924	Changed mtrNum m1 m2 to motors right left
 *	0.0.2	200920	Added movement functionality
 *	0.0.1	200919	Initial version ported from motorhat.cpp
 *
 *	Contains the class with methods to control the motors.
 *	Velocity needs to be provided in an uint16_t in the range [0 4095]
 *
 */

#include "motorcap.h"
//#include <motorhat.h>

#include <stdlib.h>
#include <math.h>

// Define below to use debug
//#define _DEBUG_MESG_
#ifdef _DEBUG_MESG_
// NOTE! Need to have compiler option -fPIC to use this
#include <iostream>		// For cout << etc... << endl
#endif

#define constrain(input,low,high) ((input)<(low)?(low):((input)>(high)?(high):(input)))

namespace embed {

//static uint8_t PWMpin[2] = {8,13};
//static uint8_t IN2pin[2] = {9,12};
//static uint8_t IN1pin[2] = {10,11};
static uint8_t __ina[2] = {1,2};
static uint8_t __inb[2] = {0,3};


MotorCAP::MotorCAP(uint8_t addr) : Pwm9685(MOTORCAP_BUSID, addr) {
	/* The Pwm9685 constructor initialize the chip and sets all outputs to zero with
	 * a Default frequency of 50 hz.
	 */
	//ToDo: Need to check if the frequency needs to be higher then the 50Hz. currently 1525
	this->setFreq(PWM_FREQUENCY);

	#ifdef _DEBUG_MESG_
	cout << " " << endl;

	cout << "MC init _ina content =";
	for ( int i =0 ; i!=2 ; i++)
		cout << "\t" << unsigned(__ina[i]) ;
	cout << endl;

	cout << "MC init _inb content =";
	for ( int i =0 ; i!=2 ; i++)
		cout << "\t" << unsigned(__inb[i]) ;
	cout << endl;

	cout << " " << endl;
	#endif
} //---[ end MotorCAP constructor ]---

MotorCAP::~MotorCAP() {
	this->~Pwm9685();
}

/*
 * Function	: runMotor
 * Purpose	: To steer a motor
 * Input	: velocity in percentage
 * Output	: ) if successful negative otherwise.
 */
int MotorCAP::runMotor(motors _motor, int8_t _velocity) {
	// Convert class to int
	int motor = (int)_motor;
	// Limit  -100 <= velocity <= 100%
	_velocity = (_velocity < -100) ? -100 : ((_velocity > 100) ? 100 : _velocity);

	if (_velocity >= 0) {
		if (this->setDutyCycle(__ina[motor], _velocity)) return -1;
		if (this->setDutyCycle(__inb[motor], 0)) return -1;
	} else {
		if (this->setDutyCycle(__ina[motor], 0)) return -1;
		if (this->setDutyCycle(__inb[motor], _velocity)) return -1;
	}

	return (0);
} //---[ end runMotor ]--

/*
 * Function	: runMotor
 * Purpose	: To steer a motor
 * Input	: velocity in percentage
 * Output	: 0 if successful negative otherwise.
 */
int MotorCAP::runMotor(motors _motor, int16_t _pcm) {
	// Convert class to int
	int motor = (int)_motor;
	// Resolution is 12 bits 0x0fff
	// Limit  -4096 < velocity < 4096
	_pcm = (_pcm < -0x0fff) ? -0x0fff : ((_pcm > 0x0fff) ? 0x0fff : _pcm);

	if (_pcm >= 0) {
		if (this->setPWMraw(__ina[motor], 0, _pcm)) return -1;
		if (this->setPWMraw(__inb[motor], 0, 0)) return -1;
	} else {
		if (this->setPWMraw(__ina[motor], 0, 0)) return -1;
		// inverse velocity
		if (this->setPWMraw(__inb[motor], 0, - _pcm)) return -1;
	}

	return (0);
} //---[ end runMotor ]--

int MotorCAP::runMotor(motors _motor, mtrCmd _command, uint8_t _velocity) {
	// Convert class to int
	int motor = (int)_motor;
	// Limit velocity to 100%
	_velocity = ( _velocity > 100) ? 100 : _velocity;

	switch(_command){
		case mtrCmd::advance:
			#ifdef _DEBUG_MESG_
			cout << "MC runMotor Advance" << endl;
			cout << "MC runMotor channel ina Channel = " << unsigned(__ina[motor]) << endl;
			cout << "MC runMotor channel inb Channel = " << unsigned(__inb[motor]) << endl;
			#endif

			if (this->setDutyCycle(__ina[motor], _velocity)) return -1;
			if (this->setDutyCycle(__inb[motor], 0)) return -1;
			break;
		case mtrCmd::reverse:
			#ifdef _DEBUG_MESG_
			cout << "MC runMotor Reverse" << endl;
			cout << "MC runMotor channel ina Channel = " << unsigned(__ina[motor]) << endl;
			cout << "MC runMotor channel inb Channel = " << unsigned(__inb[motor]) << endl;
			#endif

			if (this->setDutyCycle(__ina[motor], 0)) return -1;
			if (this->setDutyCycle(__inb[motor], _velocity)) return -1;
			break;
		//case motorCommand::release:
		default:	// Default is release, so no separate case required
			#ifdef _DEBUG_MESG_
			cout << "MC runMotor Advance" << endl;
			cout << "MC runMotor Release/Default" << endl;
			cout << "MC runMotor channel Off ina Channel = " << unsigned(__ina[motor]) << endl;
			cout << "MC runMotor channel Off inb Channel = " << unsigned(__inb[motor]) << endl;
			#endif

			if (this->channelOff(__ina[motor])) return -1;
			if (this->channelOff(__inb[motor])) return -1;
	}
	return (0);
} //---[ end runMotor ]--


int MotorCAP::stopMotor(motors _motor) {
	// Convert class to int
	int motor = (int)_motor;

	// Set velocity to zero
	if (this->setDutyCycle(__ina[motor], 0)) return -1;
	return (this->setDutyCycle(__inb[motor], 0));

} //---[ end releaseMotor ]---

int MotorCAP::releaseMotor(motors _motor) {
	// Convert class to int
	int motor = (int)_motor;

	if (this->channelOff(__ina[motor])) return -1;
	return (this->channelOff(__inb[motor]));

} //---[ end releaseMotor ]---


} //-- namespace embed --//

//===[ eof motorcap.cpp ]===//
