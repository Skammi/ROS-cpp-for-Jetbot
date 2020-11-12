/*
 *	file :			ina219.cpp
 *	Project :		jetbot
 *	Created on :	7 Nov 2020
 *	Last edit :		9 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201107	Initial version reverse engineered from python
 *
 *	Contains the class with methods to communicate with the ina219 IC.
 *
 *	A sharp load might reset the INA219, which resets the calibration register.
 *	Therefore a calibration value is always set before a current or power read.
 *	This will ensure the values read are relevant.
 */

#include "ina219.h"

namespace embed {


// Register Definitions
#define REG_CONFIG				0x00		// Config Register (R/W)
#define REG_SHUNTVOLTAGE		0x01		// Shunt voltage register (R)
#define REG_BUSVOLTAGE			0x02		// Bus voltage register (R)
#define REG_POWER				0x03		// Power register (R)
#define REG_CURRENT				0x04		// Current register (R)
#define REG_CALIBRATION			0x05		// Calibration register (R/W)

// Config register bit patterns in hex
// RST
#define POWERONRESET			0x8000		// power -on reset the defice

// BRNG BusVoltageRange bit 13 
#define RANGE_16V				0x0000		// set bus voltage range to 16V
#define RANGE_32V				0x0200		// set bus voltage range to 32V (default)

// PG  PGA bit 11&12 (Shunt Voltage Only)
#define DIV_1_40MV				0x0000		// shunt prog. gain set to  1, 40 mV range
#define DIV_2_80MV				0x0800		// shunt prog. gain set to /2, 80 mV range
#define DIV_4_160MV				0x1000		// shunt prog. gain set to /4, 160 mV range
#define DIV_8_320MV				0x1800		// shunt prog. gain set to /8, 320 mV range

// BADC bus adc resolution bit 10..7
#define BADCRES_9BIT_1S			0x0000		//  9bit,   1 sample,  84us
#define BADCRES_10BIT_1S		0x0080		// 10bit,   1 sample, 148us
#define BADCRES_11BIT_1S		0x0100		// 11 bit,  1 sample, 276us
#define BADCRES_12BIT_1S		0x0180		// 12 bit,  1 sample, 532us
#define BADCRES_12BIT_2S		0x0480		// 12 bit,  2 samples,  1.06ms
#define BADCRES_12BIT_4S		0x0500		// 12 bit,  4 samples,  2.13ms
#define BADCRES_12BIT_8S		0x0580		// 12bit,   8 samples,  4.26ms
#define BADCRES_12BIT_16S		0x0600		// 12bit,  16 samples,  8.51ms
#define BADCRES_12BIT_32S		0x0680		// 12bit,  32 samples, 17.02ms
#define BADCRES_12BIT_64S		0x0700		// 12bit,  64 samples, 34.05ms
#define BADCRES_12BIT_128S		0x0780		// 12bit, 128 samples, 68.10ms

// SADC shunt adc resolution bit 6..3
#define SADCRES_9BIT_1S			0x0000		//  9bit,   1 sample,  84us
#define SADCRES_10BIT_1S		0x0008		// 10bit,   1 sample, 148us
#define SADCRES_11BIT_1S		0x0010		// 11 bit,  1 sample, 276us
#define SADCRES_12BIT_1S		0x0018		// 12 bit,  1 sample, 532us
#define SADCRES_12BIT_2S		0x0048		// 12 bit,  2 samples,  1.06ms
#define SADCRES_12BIT_4S		0x0050		// 12 bit,  4 samples,  2.13ms
#define SADCRES_12BIT_8S		0x0058		// 12bit,   8 samples,  4.26ms
#define SADCRES_12BIT_16S		0x0060		// 12bit,  16 samples,  8.51ms
#define SADCRES_12BIT_32S		0x0068		// 12bit,  32 samples, 17.02ms
#define SADCRES_12BIT_64S		0x0070		// 12bit,  64 samples, 34.05ms
#define SADCRES_12BIT_128S		0x0078		// 12bit, 128 samples, 68.10ms

// Mode bit 2..0
#define POWERDOW				0x0000		// power down
#define SVOLT_TRIGGERED			0x0001		// shunt voltage triggered
#define BVOLT_TRIGGERED			0x0002		// bus voltage triggered
#define SANDBVOLT_TRIGGERED		0x0003		// shunt and bus voltage triggered
#define ADCOFF					0x0004		// ADC off
#define SVOLT_CONTINUOUS		0x0005		// shunt voltage continuous
#define BVOLT_CONTINUOUS		0x0006		// bus voltage continuous
#define SANDBVOLT_CONTINUOUS	0x0007		// shunt and bus voltage continuous (default)
#define CLEAR_MODE				0xFFF8		// reset bit 2..0

//======================== Private methods


/*
 * Constructor
 * input _bus number and device id
 */
Ina219::Ina219(uint8_t busId, uint8_t devId)
		: I2c(busId, devId)
{
	// Bring chip to power-on state
	this->reset();
	// Set chip to known config values
	this->calibrate32V2A();
} //---[ end uint16_t constructor ]---


Ina219::~Ina219()
{
	//nothing for now;
} //---[ end uint16_t destructor ]---

void Ina219::reset()
{
	uint16_t config = POWERONRESET;								// Set config to power on Reset
	I2c::writeRegister16b(REG_CONFIG, config);				// Write normal mode
}


void Ina219::calibrate32V2A()
{
	this->currentLsb_ = 0.1;			// Current LSB = 100 uA = 0.1 mA
	this->powerLsb_ = 2.0;				// Power LSB = 2 mW
	this->calibrationValue_ = 4096;		// = trunc (0.04096 / (Current_LSB{0.0001 * RShunt{0.1})
										// cal = trunc (0.04096 /0.00001) = 4096
	I2c::writeRegister16b(REG_CALIBRATION, this->calibrationValue_);

	// calculate config register
	uint16_t config = RANGE_32V | DIV_8_320MV | BADCRES_12BIT_1S |
						SADCRES_12BIT_1S | SANDBVOLT_CONTINUOUS;
	I2c::writeRegister16b(REG_CONFIG, config);
}

void Ina219::calibrate32V1A()
{
	this->currentLsb_ = 0.04;			// Current LSB = 40 uA = 0.04 mA
	this->powerLsb_ = 0.8;				// Power LSB = 800 uW
	this->calibrationValue_ = 10240;	// = trunc (0.04096 / (Current_LSB{0.00004 * RShunt{0.1})
										// cal = trunc (0.04096 /0.000004) = 10240
	I2c::writeRegister16b(REG_CALIBRATION, this->calibrationValue_);

	// calculate config register
	uint16_t config = RANGE_32V | DIV_8_320MV | BADCRES_12BIT_1S |
						SADCRES_12BIT_1S | SANDBVOLT_CONTINUOUS;
	I2c::writeRegister16b(REG_CONFIG, config);
}


void Ina219::calibrate16V400mA()
{
	this->currentLsb_ = 0.05;			// Current LSB = 50 uA = 0.05 mA
	this->powerLsb_ = 1.0;				// Power LSB = 1 mW
	this->calibrationValue_ = 8192;		// = trunc (0.04096 / (Current_LSB{0.00005 * RShunt{0.1})
										// cal = trunc (0.04096 /0.000005) = 8192
	I2c::writeRegister16b(REG_CALIBRATION, this->calibrationValue_);

	// calculate config register
	uint16_t config = RANGE_16V | DIV_1_40MV | BADCRES_12BIT_1S |
						SADCRES_12BIT_1S | SANDBVOLT_CONTINUOUS;
	I2c::writeRegister16b(REG_CONFIG, config);
}


void Ina219::powerSave(bool _PowerSaveOn)
{
	uint16_t config = I2c::readRegister16b(REG_CONFIG);			// read the current config
	config = config & CLEAR_MODE;								// set bit 0..2 to 0
	if (_PowerSaveOn) {
		config = config | POWERDOW;
		I2c::writeRegister16b(REG_CONFIG, config);				// Write Power save
	} else {
		config = config | SANDBVOLT_CONTINUOUS;
		I2c::writeRegister16b(REG_CONFIG, config);				// Write normal mode
	}
}

float Ina219::getShuntVoltage()
{
	uint16_t value = I2c::readRegister16b(REG_SHUNTVOLTAGE);	// read the shunt voltage
	return (value * shuntVoltLsb_);								// return value in mV
}


float Ina219::getBusVoltage()
{
	uint16_t value = I2c::readRegister16b(REG_BUSVOLTAGE);		// read the bus voltage (mV)
	// Shift to the right 3 to drop CNVR and OVF
	value = (value >> 3);  // Note can be pushed in return statement
	return (value * this->voltageLsb_);							// return value in V
}


float Ina219::getCurrent()
{
	// Set the calibration register.
	I2c::writeRegister16b(REG_CALIBRATION, this->calibrationValue_);
	uint16_t value = I2c::readRegister16b(REG_CURRENT);			// read the current
	return (value * currentLsb_);								// return value in mA
}


float Ina219::getPower()
{
	// Set the calibration register.
	I2c::writeRegister16b(REG_CALIBRATION, this->calibrationValue_);
	uint16_t value = I2c::readRegister16b(REG_POWER);			// read the power
	return (value * powerLsb_);									// return value in mW
}


//======================== Private methods



} /* namespace embed */

//===[ eof ina219.cpp ]===//

