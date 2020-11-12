/*
 *	file :			i2cjtsn.h
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


#ifndef _I2C_H_
#define _I2C_H_
#define I2CBUS_0 "/dev/i2c-0"	// First I2C interface
#define I2CBUS_1 "/dev/i2c-1"	// Second I2C interface
#include <cstdint>

namespace embed {


class I2c {
private:
	uint8_t cBusId {0xFF};
	uint8_t cDevId {0xFF};
	int cFileHandle {-1};
public:
	// Create instance and open handle to device
	I2c(uint8_t busId, uint8_t deviceId);
	int open();

	int getHandle();
	void setHandle(int fileHandle);
	int writeByte(uint8_t value);
	unsigned char readRegister(uint8_t registerAddress);
	uint16_t readRegister16b(uint8_t registerAddress);
	unsigned char* readRegisters(uint8_t number, uint8_t fromAddress=0);
	int writeRegister(uint8_t registerAddress, uint8_t value);
	int writeRegister16b(uint8_t registerAddress, uint16_t value);
	void close();
	~I2c();
}; //-- class I2c --//


} /* end namespace embed */

#endif //-- _I2C_H_ --//

//===[ eof i2c.h ]===//
