/*
 *	file :			rpmsensor.h
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
 */


#ifndef _RPMSENSOR_H_
#define _RPMSENSOR_H_

#include <stdio.h>
#include <unistd.h>
#include <i2c.h>

using namespace std;

namespace embed {


#define RPMSENSOR_BUSID		1
#define RPMSENSOR_DEVID		0x11

//#define RPM_A_REG     0x01     // Register Address of speed sensor A
//#define RPM_B_REG     0x02     // Register Address of speed sensor B

//			rpmRegister {0, 1 ,2, 3};
enum class rpmRegister {conf, rpm_a, rpm_b, rpm_ab};


class RpmSensor : protected I2c {

public:
	RpmSensor(uint8_t addr=RPMSENSOR_DEVID);
	~RpmSensor();

	uint16_t readRpmRegister(rpmRegister _reg);
	uint16_t* readRpmRegisters(rpmRegister _reg);
private:
	// nothing for now
};

} //-- namespace embed --//

#endif //-- _RPMSENSOR_H_ --//

