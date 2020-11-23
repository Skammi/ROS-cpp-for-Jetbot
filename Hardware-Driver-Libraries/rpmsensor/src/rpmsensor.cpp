/*
 *	file :			rpmsensor.cpp
 *	Project :		jetbot
 *	Created on :	15 Nov 2020
 *	Last edit :		22 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.2	201122	Added method to read both registers
 *	0.0.1	201115	Initial version
 *
 *	Contains the class with methods to read the RPM Sensors.
 *	RPM sensor returns 16 bit value. 0xFFFF is error response
 *
 */

#include "rpmsensor.h"


#include <stdlib.h>
#include <math.h>

// Define below to use debug
#define _DEBUG_MESG_
#ifdef _DEBUG_MESG_
// NOTE! Need to have compiler option -fPIC to use this
#include <iostream>		// For cout << etc... << endl
#endif


namespace embed {


/*
 * Constructor
 *  device id
 */
RpmSensor::RpmSensor(uint8_t addr) : I2c(RPMSENSOR_BUSID, addr)
{
	//nothing for now;
} //---[ end constructor ]---


RpmSensor::~RpmSensor() {
	this->~I2c();
} //---[ end destructor ]---


/*
 * Function	: readRpmRegister
 * Purpose	: To read the rpm from a wheel
 * Input	: rpmRegister
 * Output	: RPM if successful, 0xFFFF when error.
 */
uint16_t RpmSensor::readRpmRegister(rpmRegister _reg) {
	// Convert class to unin_t
	int reg = (uint8_t)_reg;
	uint16_t rpm = I2c::readRegister16b(reg);
	return (rpm);
} //---[ end readRpmRegister ]--


/*
 * Function	: readRpmRegister
 * Purpose	: To read the rpms from both wheels
 * Input	: rpmRegister
 * Output	: RPM if successful, 0xFFFF when error.
 */
uint16_t* RpmSensor::readRpmRegisters(rpmRegister _reg) {
	// Convert class to unin_t
	int reg = (uint8_t)_reg;

	unsigned char *buffer;		// Create a pointer a byte buffer
	static uint16_t values[2];	// Read 2 16bit registers
	buffer = I2c::readRegisters(4, reg);	// read 4 bytes
	values[0] = (buffer[0] << 8) + buffer[1];
	values[1] = (buffer[2] << 8) + buffer[3];
	return (values);
} //---[ end readRpmRegisters ]--

} //-- namespace embed --//

//===[ eof rpmsensor.cpp ]===//
