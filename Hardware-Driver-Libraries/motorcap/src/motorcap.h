/*
 *	file :			motorcap.h
 *	Project :		jetbot
 *	Created on :	19 Sep 2020
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


#ifndef _MOTORCAP_H_
#define _MOTORCAP_H_

#include <stdio.h>
#include <unistd.h>
#include <pwm9685.h>

using namespace std;

namespace embed {


#define MOTORCAP_BUSID		1
#define MOTORCAP_DEVID		0x60

#define PWM_FREQUENCY			1525
//			motors {0, 1};
enum class motors {right, left};
//			motorCmd {0, 1, 2};
enum class mtrCmd {advance, reverse};

class MotorCAP : public Pwm9685 {

private:

public:
	//static uint16_t Hat_Addr[4];
	MotorCAP(uint8_t addr=MOTORCAP_DEVID);
	~MotorCAP();

	int runMotor(motors _motor, int8_t _velocity);
	int runMotor(motors _motor, int16_t _pcm);
	int runMotor(motors _motor, mtrCmd _command, uint8_t _velocity);
	int stopMotor(motors _motor);
	int releaseMotor(motors _motor);

private:
	// nothing for now
};

} //-- namespace embed --//

#endif //-- _MOTORCAP_H_ --//
