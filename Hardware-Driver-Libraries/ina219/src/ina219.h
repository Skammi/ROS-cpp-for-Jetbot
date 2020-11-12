/*
 *	file :			ina219.h
 *	Project :		jetbot
 *	Created on :	7 Nov 2020
 *	Last edit :		9 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201107	Initial version
 *
 *	Contains the class with methods to communicate with the ina219 IC.
 *
 */

#ifndef INA219_H_
#define INA219_H_

#include <i2c.h>

namespace embed {


class Ina219 : public I2c
{
	public:
		Ina219(uint8_t _busId, uint8_t _devId);
		~Ina219();

		void reset();
		void calibrate32V2A();
		void calibrate32V1A();
		void calibrate16V400mA();
		void powerSave(bool _PowerSaveOn);

		float getShuntVoltage();
		float getBusVoltage();
		float getCurrent();
		float getPower();

	private:
		// fixed values
		const float shuntVoltLsb_ {0.01};		// mV ,	Shunt Voltage LSB = 10 uV
		const float voltageLsb_ {0.004};		// V ,	voltage LSB = 4 mV = 0.004 V
		// Default values, they are also set in methode ...32V2A
		float currentLsb_ {0.1};				// mA ,	Current LSB = 100 uA
		float powerLsb_ {2.0};					// mW ,	Power LSB = 2 mW
		uint16_t calibrationValue_ {4096};		// = trunc (0.04096 / (Current_LSB{0.0001 * RShunt{0.1})
												// cal = trunc (0.04096 /0.00001) = 4096

}; // end class Ina219

} /* end namespace embed */

#endif /* INA219_H_ */

