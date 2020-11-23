/*
 *	file :			rpmsensor_poc_main.cpp
 *	Project :		jetbot
 *	Created on :	15 Nov 2020
 *	Last edit :		22 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.2	201122	Added read both registers
 *	0.0.1	201115	Initial version
 *
 */


#include "rpmsensor_poc.h"

void readSensors (RpmSensor *_rpmSensor)
{
	uint16_t rpm {0};
	rpm = _rpmSensor->readRpmRegister(rpmRegister::rpm_a);
	cout << "RpmSensor POC: Sensor A RPM: " << rpm << endl;
	sleep (1);		// Wait a bit.

	rpm = _rpmSensor->readRpmRegister(rpmRegister::rpm_b);
	cout << "RpmSensor POC: Sensor B RPM: " << rpm << endl;
	sleep (1);		// Wait a bit.

	rpm = _rpmSensor->readRpmRegister(rpmRegister::conf);
	cout << "RpmSensor POC: Conf content: " << rpm << endl;
	sleep (1);		// Wait a bit.

	uint16_t *rpms;
	rpms = _rpmSensor->readRpmRegisters(rpmRegister::rpm_ab);
	cout << "RpmSensor POC: Sensor A content: " << rpms[0] << endl;
	cout << "RpmSensor POC: Sensor B content: " << rpms[1] << endl;

}



int main()
{
	cout << "RpmSensor POC: Test Concept V0.0.1!" << endl;
	int noftest		{5};	// Perform default 5 tests
	int timedelay	{5};	// Wait default 5 seconds

	// Initiate an instance of motorHat
	RpmSensor *rpmSensor = new RpmSensor(RPMSENSOR_DEVID);

	cout << "RpmSensor POC: RpmSensor Initiated!" << endl;


	// Run the tests
	for (int i=0; i < noftest ; i++ )
	{
		readSensors (rpmSensor);
		sleep (timedelay);		// Wait a bit.
	};
	readSensors (rpmSensor);

	// Clean up.
	rpmSensor->~RpmSensor();

	return 0;
} //---[ end main ]---


//===[ eof motorcap_poc_main.cpp ]===//
