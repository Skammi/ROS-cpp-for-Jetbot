/*
 *	file :			ina219_poc_main.cpp
 *	Project :		jetbot
 *	Created on :	9 Nov 2020
 *	Last edit :		9 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201109	Initial version.
 *
 */
/*
#define INAJETBOT_BUSID		1
#define INAJETBOT_DEVID		0x60
*/

#include "ina219_poc.h"
/************************************************************************************************************
 *
 * Main routine
 *
 *
 */

int main(int argc, char* argv[]) {

	cout << "ina219 POC: Test Concept V0.0.1!" << endl;

	// Initiate an instance of Ina219 class
	Ina219 *ina = new Ina219(INAJETBOT_BUSID, INAJETBOT_DEVID);
	cout << "ina219 POC: ina219 Initiated!" << endl;


	// Run the tests

	cout << "ina219 POC: Read Current!" << endl;
	float current = ina->getCurrent();
	cout << "ina219 POC: Current = " << current << " mA." << endl;

	cout << "ina219 POC: Read Power!" << endl;
	float power = ina->getPower();
	cout << "ina219 POC: Power = " << power << " mW." << endl;

	cout << "ina219 POC: Read Voltage!" << endl;
	float voltage = ina->getBusVoltage();
	cout << "ina219 POC: Voltage = " << voltage << " V." << endl;

	cout << "ina219 POC: Read Shunt Voltage!" << endl;
	float shuntVoltage = ina->getShuntVoltage();
	cout << "ina219 POC: Shunt Voltage = " << shuntVoltage << " mV." << endl;

	return 0;
} //---[ end main ]---

//===[ eof ina219_poc_main.cpp ]===//


