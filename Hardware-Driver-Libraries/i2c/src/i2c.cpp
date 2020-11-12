/*
 *	file :			i2c.cpp
 *	Project :		jetbot
 *	Created on :	14 Sep 2020
 *	Last edit :		9 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.2	201108	Added 16bit data read and write
 *	0.0.1	200914	Initial version ported from i2crpi v0.0.6
 *
 */
#include<iostream>
#include<sstream>
#include<fcntl.h>
#include<stdio.h>
#include<iomanip>
#include<unistd.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>

#include "i2c.h"
using namespace std;

#define HEX(x) setw(2) << setfill('0') << hex << (int)(x)

namespace embed {

/*
 * I2c
 */
I2c::I2c(uint8_t _busId, uint8_t _deviceId) {
	cBusId = _busId;
	cDevId = _deviceId;
	this->open();
}

int I2c::open() {
	// Set name string  x = (condition) ? (value_if_true) : (value_if_false)
	string name = cBusId==0 ? I2CBUS_0 : I2CBUS_1;
	//string name = "/dev/i2c-1";

	if((cFileHandle=::open (name.c_str(), O_RDWR)) < 0) {
		return -1;
	}
	if(ioctl(cFileHandle, I2C_SLAVE, cDevId) < 0) {
		return -2;
	}
	return 0;
}

int I2c::getHandle() {
	return cFileHandle;
}

void I2c::setHandle(int fileHandle) {
	cFileHandle =fileHandle;
}

int I2c::writeByte(uint8_t value){
	unsigned char buffer[1];
	buffer[0]=value;
	if (::write(cFileHandle, buffer, 1) != 1)
		return -3;
	return 0;
}

unsigned char I2c::readRegister(uint8_t registerAddress){
	this->writeByte(registerAddress);
	unsigned char buffer[1];
	if(::read(cFileHandle, buffer, 1) != 1)
		return -4;

	return buffer[0];
}
uint16_t I2c::readRegister16b(uint8_t registerAddress){
	this->writeByte(registerAddress);
	uint8_t* buffer = new uint8_t[2];
	if(::read(cFileHandle, buffer, 2) != 2)
		return -4;
	// note			msb 1st data byte	lsb 2nd data byte
	uint16_t value = (buffer[0] << 8) + buffer[1];
	return value;
}

unsigned char* I2c::readRegisters(uint8_t number, uint8_t fromAddress){
	this->writeByte(fromAddress);
	uint8_t* data = new uint8_t[number];
	if(::read(cFileHandle, data, number) != (int)number)
    	return NULL;
	return data;
}

int I2c::writeRegister(uint8_t registerAddress, uint8_t value){
	uint8_t buffer[2];
	buffer[0] = registerAddress;
	buffer[1] = value;
	if(::write(cFileHandle, buffer, 2)!=2){
		return -5;
	}
	return 0;
}

int I2c::writeRegister16b(uint8_t registerAddress, uint16_t value){
	uint8_t buffer[3];
	buffer[0] = registerAddress;
	buffer[1] = value >> 8;			// msb in first data byte
	buffer[2] = value & 0xFF;		// lsb in second data byte
	if(::write(cFileHandle, buffer, 3)!=3){
		return -5;
	}
	return 0;
}

void I2c::close() {
	::close(cFileHandle);
	cFileHandle = -1;
}

I2c::~I2c() {
	if(cFileHandle) this->close();
}


} /* namespace embed */

//===[ eof I2.cpp ]===//

