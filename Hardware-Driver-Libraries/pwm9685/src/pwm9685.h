/*
 *	file :			pwm9685.h
 *	Project :		JetBot
 *	Created on :	15 Sep 2020
 *	Last edit :		16 Sep 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.2	200921	Initial version, ported from Pca9685.h v0.0.7.
 *	0.0.1	200915	Initial version, ported from Pca9685.h v0.0.7.
 *
 *	Contains the Class and methods to use a pca9685 as PWM:
 *		Sweeping between 2 values.
 *
 */

#ifndef _PWM9685_H
#define _PWM9685_H
#include <cstdint>
#include <i2c.h>

namespace embed {


#define PWMRESOLUTION	4095

// Minimum Maximum values
#define MINPWMFREQ		24		// Hz. Minimum PWM frequency
#define MAXPWMFREQ		1526	// Hz. Maximum PWM frequency

//#define MINPWMANGLE		0		// degr. Minimum PWM angle
#define MAXPWMANGLE		180		// degr. Maximum PWM angle
#define MINPWMPULSE		500		// uSec. Minimum PWM pulse
#define MAXPWMPULSE		2500	// uSec. Maximum PWM pulse

// Default values
#define DEFPWMFREQ		50		// Hz. Default PWM frequency

class Pwm9685 : public I2c {
public:
	// Creates an object to initialize board.
	Pwm9685(uint8_t _busId, uint8_t _devId);
	//Pwm9685(uint16_t busId, uint16_t devId);

	void resetPwm(void);
	void detachPwm(void);
	int sleepPwm(void);
	int setFreq(uint16_t _freq);

	int channelOn(uint8_t _channel);
	int channelOff(uint8_t _channel);

	int setPulse(uint8_t _channel, uint16_t _pulse);
	int setAngle(uint8_t _channel, uint16_t _angle);
	int setDutyCycle(uint8_t _channel, uint8_t _dutyCycle);
	int setToff(uint8_t _channel, uint16_t _toff);
	int setPWMraw(uint8_t _channel, uint16_t _ton, uint16_t _toff);
	~Pwm9685();
private:
	uint8_t channelOffset(uint8_t _channel);
}; // end class Pcwm9685

} /* end namespace embed */

#endif	/* _PWM9685_H */

//===[ eof pwm9685.h ]===//
