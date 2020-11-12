/*
 *	file :			pwm9685.cpp
 *	Project :		JetBot
 *	Created on :	15 Sep 2020
 *	Last edit :		30 Sep 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.2	200925	Added channel on and off commands.
 *	0.0.1	200915	Initial version, ported from Pca9685.h v0.0.7.
 *
 *	Contains the Class and methods to use a pca9685 for PWM:
 *		Sweeping between 2 values.
 *
 */

//ToDo: implement sleep function!
#include <unistd.h>
#include "pwm9685.h"

// Define below to use debug
//#define _DEBUG_MESG_
#ifdef _DEBUG_MESG_
// NOTE! Need to have compiler option -fPIC to use this
#include <iostream>		// For cout << etc... << endl
using namespace std;

#endif

//===[ Program Macro's ]===??

#define constrain(input,low,high) ((input)<(low)?(low):((input)>(high)?(high):(input)))

namespace embed {

// Register Definitions
#define MODE1			0x0		// Mode register 1
#define MODE2			0x1		// Mode register 2
#define SUBADR1			0x2		// I2C bus sub address 1
#define SUBADR2			0x3		// I2C bus sub address 2
#define SUBADR3			0x4		// I2C bus sub address 3

#define CHNON_L			0x6		// Channel 0 ON LSB
#define CHNON_H			0x7		// Channel 0 ON MSB
#define CHNOFF_L		0x8		// Channel 0 OFF LSB
#define CHNOFF_H		0x9		// Channel 0 OFF MSB
#define CHNMULTIP		4		// channel 5 ton l = CHNON_L(0x6) + 5 * 4 = 0x1A
								// channel 0 toff l = CHNOFF_H(0x9) + 0 * 4 = 0x09
#define ON_L			0x0		// Channel ON LSB
#define ON_H			0x1		// Channel ON MSB
#define OFF_L			0x2		// Channel OFF LSB
#define OFF_H			0x3		// Channel OFF MSB


#define ALL_CHNON_L		0xFA	// All channels ON duty cycle LSB
#define ALL_CHNON_H		0xFB	// All channels ON duty cycle MSB
#define ALL_CHNOFF_L	0xFC	// All channels OFF duty cycle LSB
#define ALL_CHNOFF_H	0xFD	// All channels OFF duty cycle MSB

#define PRE_SCALE		0xFE	// pre-scaler for PWM output frequency


/*
 * Pre-scale value = round (Clock_freq / (4096 * freq)) - 1
 *  Clock_Freq ~ 25 Mhz
 *  Clock_freq / 4096 => 25,000,000 / 4096 = 6103.51.
 * Pre-scale value = round (6103.51 / freq)) - 1
 *
 * As we do not want to use round we force the result in an uint8
 * Pre-scale value = (uint16_t) (6103.51 / freq)) - 0.5
 *
 */
#define PRESCALECONST	6103.51
// Modes
#define NORMALMODE		0x00	// normal
#define SLEEPMODE		0x10	// sleep set bit 4
#define RESTARTMODE		0x80	// restart set bit 7
#define DEFAULTMODE		0xA1	// binary 1010 0001
								// b7 1 reset
								// b5 1 register auto increment
								// b1 1 Respond to all call
#define OCHONACK		0x04	// Output Change On Ack, all regs set


/*
 * Class Pwm9685
 */

/*
 * Constructor
 * input _bus number and device id
 */
Pwm9685::Pwm9685(uint8_t busId, uint8_t devId)
		: I2c(busId, devId) {
	// Bring the device in reset mode.
	this->resetPwm();
	// Set the default Signal Frequency
	this->setFreq(DEFPWMFREQ);
	// write 1 to bit 4 of ALL_CHNOFF_H -> all outputs 0
	this->writeRegister(ALL_CHNOFF_H, 0x10);

	//cout << "Pca9685: Construction completed! bus: " << busId << " Device: " << devId << endl;
} //---[ end Pwm9685 constructor ]---


Pwm9685::~Pwm9685() {
	this->detachPwm();
} //---[ end Pwm9685 destructor ]---

void Pwm9685::detachPwm() {
	this->resetPwm();
	// write 1 to bit 4 of ALL_CHNOFF_H -> all outputs 0
	this->writeRegister(ALL_CHNOFF_H, 0x10);
} //---[ end detach ]---

// Set PCA9685 mode to Default
void Pwm9685::resetPwm() {
	this->writeRegister(MODE1, NORMALMODE);
	this->writeRegister(MODE2, OCHONACK);
	//cout << "Pca9685: Device reseted!" << endl;
} //---[ end reset ]---

uint8_t Pwm9685::channelOffset(uint8_t channel) {
	return (CHNON_L + (CHNMULTIP * channel));
} //---[ end  ]---

/*
 * Set the frequency of PWM
 *
 * Input frequency. 40 - 1000 (Hz).
 */
int Pwm9685::setFreq(uint16_t _freq) {
	// exit if 24 <= _freq < 1526
	if (MINPWMFREQ <= _freq && _freq < MAXPWMFREQ)
		return -11;
	uint16_t prescaleVal = (uint16_t)((PRESCALECONST/_freq)-0.5);

	if (this->writeRegister(MODE1, SLEEPMODE))
		return -12;
	if (this->writeRegister(PRE_SCALE, prescaleVal))
		return -13;
	// let oscillator stabilize
	usleep (500);
	// Reset the IC.
	this->resetPwm();
	return 0;
} //---[ end  ]---


int Pwm9685::sleepPwm(void) {
	return 0;
} //---[ end  ]---

/*
 * Function	: channelOn
 * Purpose	: Set the Channel high
 * Input	: Channel no
 * Output	: 0 ok !0 error
 *
 *	for the output to be HIGH bit 4 of LEDn_ON_H must be HIGH and bit 4 of LEDn_OFF_H must be LOW
 *	for the output to be LOW bit 4 of LEDn_ON_H must be LOW and bit 4 of LEDn_OFF_H must be HIGH
 *	As we are not interested in the other bits we can just write 0x10 to make it HIGH an 0x00 to make it LOW
 *
 */
int Pwm9685::channelOn(uint8_t _channel) {

	uint8_t ChanRegOffset = this->channelOffset( _channel);
	/***
	#ifdef _DEBUG_MESG_
	uint8_t before {0};
	uint8_t after {0};

	before = this->readRegister(ChanRegOffset + ON_H); // CHN n ON_H
	cout << "Pca9685: channelOn: before set bit 4 on ON_H: " << unsigned(before) << endl;
	#endif
	// write 0 to bit 4 of OFF_H -> output NOT low
	if (this->writeRegister(ChanRegOffset + OFF_H, 0x00)) return(-1);; // CHN n OFF_H reset bit4 (1 means output low
	#ifdef _DEBUG_MESG_
	after = this->readRegister(ChanRegOffset + ON_H); // CHN n ON_H
	cout << "Pca9685: channelOn: after set bit 4 on ON_H: " << unsigned(after) << endl;

	before = this->readRegister(ChanRegOffset + OFF_H); // CHN n ON_H
	cout << "Pca9685: channelOn: before reset bit 4 on ON_H: " << unsigned(before) << endl;
	#endif

	// write 1 to bit 4 of ON_H -> output high
	if (this->writeRegister(ChanRegOffset + ON_H, 0x10)) return(-1);  // CHN n ON_H set bit 4
	#ifdef _DEBUG_MESG_
	after = this->readRegister(ChanRegOffset + OFF_H); // CHN n OFF_H
	cout << "Pca9685: channelOn: after reset bit 4 on OFF_H: " << unsigned(after) << endl;
	#endif

	return(0);
	 ***/


	// write 0 to bit 4 of OFF_H -> output NOT low
	if (this->writeRegister(ChanRegOffset + OFF_H, 0x00)) return -1;	// CHN n OFF_H reset bit4
	// write 1 to bit 4 of ON_H -> output high
	return (this->writeRegister(ChanRegOffset + ON_H, 0x10));			// CHN n ON_H set bit 4

} //---[ end channelOn ]---


int Pwm9685::channelOff(uint8_t _channel) {
	uint8_t ChanRegOffset = this->channelOffset( _channel);
	// write 0 to bit 4 of ON_H -> outputs NOT high
	if (this->writeRegister(ChanRegOffset + ON_H, 0x00)) return(-1);	// CHN n ON_H reset bit 4
	// write 1 to bit 4 of OFF_H -> outputs low
	return (this->writeRegister(ChanRegOffset + OFF_H, 0x10));			// CHN n OFF_H set bit 4
} //---[ end channelOff ]---

/*
 * Set the the Pulse time
 *
 * Pulse in uSec (??)
 */
int Pwm9685::setPulse(uint8_t _channel, uint16_t _pulse) {
	// Constrain input between Minimum and maximum
	_pulse = constrain (_pulse, MINPWMPULSE, MAXPWMPULSE);

	#ifdef _DEBUG_MESG_
	cout << "Pca9685: set Channel:" << _channel << " Pulse: " << _pulse << endl;
	#endif

	uint8_t ChanRegOffset = this->channelOffset( _channel);

	// Always start at begin of period so Channel On register L and H are both 0
	if (this->writeRegister(ChanRegOffset, 0))							// CHN n ON_L
		return -14;
	if (this->writeRegister(ChanRegOffset + ON_H, 0))					// CHN n ON_H
		return -15;
	if (this->writeRegister(ChanRegOffset + OFF_L, _pulse & 0xFF))		// CHN n OFF_L
		return -16;
	return (this->writeRegister(ChanRegOffset + OFF_H, _pulse >> 8));	// CHN n OFF_H
} //---[ end setPulse ]---


/*
 * Set the Angle
 *
 * Input channel number (0-15), Angle in degr. (0-180)
 */
int Pwm9685::setAngle(uint8_t _channel, uint16_t _angle) {
	// Limit Toff to 4095
	_angle = (_angle > MAXPWMANGLE) ? MAXPWMANGLE : _angle;

	//cout << "Pca9685: set angle channel: " << channel << " angle: " << angle << endl;
	#ifdef _DEBUG_MESG_
	cout << "Pca9685: set Channel:" << _channel << " Angle: " << _angle << endl;
	#endif

	uint8_t ChanRegOffset = this->channelOffset( _channel);

	// Always start at begin of period so Channel On register L and H are both 0
	if (this->writeRegister(ChanRegOffset, 0))				// CHN n ON_L
		return -18;
	if (this->writeRegister(ChanRegOffset + ON_H, 0))			// CHN n ON_H
		return -19;
	// Calculate the Channel off register value.
	unsigned int chanOff = MAXPWMPULSE - (_angle * (MAXPWMPULSE - MINPWMPULSE) / MAXPWMANGLE);
	if (this->writeRegister(ChanRegOffset + OFF_L, chanOff & 0xFF))		// CHN n OFF_L
		return -20;
	return (this->writeRegister(ChanRegOffset + OFF_H, chanOff >> 8));	// CHN n OFF_H
} //---[ end setAngle ]---


/*
 * Set the the Ton time
 *
 * Input channel number (0-15), duty cycle in % (0-100)
 */
int Pwm9685::setDutyCycle(uint8_t _channel, uint8_t _dutyCycle) {
	// Limit Duty Cycle to 100%
	_dutyCycle = (_dutyCycle > 100)? 100 : _dutyCycle;

	#ifdef _DEBUG_MESG_
	cout << "Pca9685: set Channel:" << _channel << " DutyCycle: " << _dutyCycle << endl;
	#endif

	uint8_t ChanRegOffset = this->channelOffset( _channel);

	// Always start at begin of period so Channel On register L and H are both 0
	if (this->writeRegister(ChanRegOffset, 0))			// CHN n ON_L delay is 0
		return -22;
	if (this->writeRegister(ChanRegOffset + ON_H, 0))	// CHN n ON_H delay is 0
		return -23;
	// Calculate the Channel off register value.

	int chanOff = _dutyCycle * PWMRESOLUTION / 100;

	if (this->writeRegister(ChanRegOffset + OFF_L, chanOff & 0xFF))	// CHN n OFF_L
		return -24;
	return (this->writeRegister(ChanRegOffset + OFF_H, chanOff >> 8));	// CHN n OFF_H
} //---[ end setDutyCycle ]---

/*
 * Set the the Ton time
 *
 * Input channel number (0-15), duty cycle in % (0-100)
 */
int Pwm9685::setToff(uint8_t _channel, uint16_t _toff) {
	// Limit Toff to 4095
	_toff = (_toff >PWMRESOLUTION)? PWMRESOLUTION : _toff;

	#ifdef _DEBUG_MESG_
	cout << "Pca9685: set Channel:" << _channel << " T_OFF: " << _toff << endl;
	#endif

	uint8_t ChanRegOffset = this->channelOffset( _channel);

	// Always start at begin of period so Channel On register L and H are both 0
	if (this->writeRegister(ChanRegOffset, 0))			// CHN n ON_L delay is 0
		return -22;
	if (this->writeRegister(ChanRegOffset + ON_H, 0))		// CHN n ON_H delay is 0
		return -23;
	// Calculate the Channel off register value.

	if (this->writeRegister(ChanRegOffset + OFF_L, _toff & 0xFF))	// CHN n OFF_L
		return -24;
	return (this->writeRegister(ChanRegOffset + OFF_H, _toff >> 8));	// CHN n OFF_H
} //---[ end setDutyCycle ]---
/*
 * Set the the Ton time
 *
 * Input channel number (0-15), duty cycle in % (0-100)
 */
int Pwm9685::setPWMraw(uint8_t _channel, uint16_t _ton, uint16_t _toff) {
	// no Limits its raw!!!!

	#ifdef _DEBUG_MESG_
	cout << "Pca9685: set Raw Channel:" << _channel << " T_ON: " << _ton << " T_OFF: " << _toff << endl;
	#endif

	uint8_t ChanRegOffset = this->channelOffset( _channel);

	// Always start at begin of period so Channel On register L and H are both 0
	if (this->writeRegister(ChanRegOffset, _ton))			// CHN n ON_L delay is 0
		return -22;
	if (this->writeRegister(ChanRegOffset + ON_H, _ton))		// CHN n ON_H delay is 0
		return -23;
	// Calculate the Channel off register value.

	if (this->writeRegister(ChanRegOffset + OFF_L, _toff & 0xFF))	// CHN n OFF_L
		return -24;
	return (this->writeRegister(ChanRegOffset + OFF_H, _toff >> 8));	// CHN n OFF_H
} //---[ end setDutyCycle ]---

} /* namespace embed */

//===[ eof pwm9685.cpp ]===//
