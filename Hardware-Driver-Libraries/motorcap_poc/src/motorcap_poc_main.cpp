/*
 *	file :			motorcap_poc_main.cpp
 *	Project :		jetbot
 *	Created on :	19 Sep 2020
 *	Last edit :		30 Sep 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.4	200930	Adapted to additions in MotorCAP class using version v0.0.6.
 *	0.0.3	200930	Adapted to changes in MotorCAP class using version v0.0.5.
 *	0.0.2	200924	Adapted to change from: mtrNum m1 m2 to motors right left.
 *	0.0.1	200919	Initial version.
 *
 */

#include "motorcap_poc.h"

/***
int testAdvance(MotorCAP *mtrHt) {
	cout << "MotorCap POC: testAdvance Start!" << endl;

	// set motor speed to ~25%
	uint8_t velocity {25};

	// set motor command to Advance
	if (mtrHt->runMotor(motors::right, mtrCmd::advance, velocity)) return -43;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance, velocity)) return -44;
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;
	sleep (1);		// Wait amount sec to see the action.

	// set motor velocity to ~50%
	velocity = 50;
	if (mtrHt->runMotor(motors::right, mtrCmd::advance, velocity)) return -43;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance, velocity)) return -44;
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;
	sleep (3);			// Wait 1 sec to see the action.

	// set motor velocity to ~75%
	velocity = 75;
	if (mtrHt->runMotor(motors::right, mtrCmd::advance, velocity)) return -43;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance, velocity)) return -44;
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;
	sleep (3);			// Wait 1 sec to see the action.

	// set motor velocity to ~100%
	velocity = 100;
	if (mtrHt->runMotor(motors::right, mtrCmd::advance, velocity)) return -43;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance, velocity)) return -44;
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;
	sleep (3);			// Wait 1 sec to see the action.


	// set motor velocity to 0%
	velocity = 0;
	if (mtrHt->runMotor(motors::right, mtrCmd::advance, velocity)) return -43;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance, velocity)) return -44;
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;

	sleep (1);			// Wait 1 sec to see the action.
	if (mtrHt->releaseMotor(motors::right)) return -47;
	if (mtrHt->releaseMotor(motors::left)) return -48;
	cout << "MotorCap POC: testAdvance! cmd release set" << endl;

	return 0;
} //---[ testAdvance ]---
***/

/***

int testMrun(MotorCAP *mtrHt, motors M) {
	cout << endl << "MotorCap POC: Mrun Start!" << endl;

	uint16_t velocity {0};
	// set motor speed to 20%
	if (mtrHt->setVelocity(M,velocity)) return -41;
	cout << "MotorCap POC: Mrun! Velo set to: " << velocity << endl;

	// set motor command to Advance
	if (mtrHt->runMotor(M, mtrCmd::advance)) return -44;
	cout << "MotorCap POC: Mrun! cmd advance set" << endl;

	sleep(SLPTM );		// Wait amount sec to see the action.

	if (mtrHt->runMotor(M, mtrCmd::release)) return -48;
	cout << "MotorCap POC: Mrun! cmd release set" << endl;

	sleep(3);

	if (mtrHt->runMotor(M, mtrCmd::reverse)) return -44;
	cout << "MotorCap POC: Mrun! cmd reverse set" << endl;

	sleep(SLPTM );		// Wait amount sec to see the action.

	if (mtrHt->runMotor(M, mtrCmd::release)) return -48;
	cout << "MotorCap POC: Mrun! cmd release set" << endl;

	return 0;
} //---[ testMrun ]---

int testRun(MotorCAP *mtrHt ) {
	cout << endl << "MotorCap POC: Mrun Start!" << endl;

	uint16_t velocity {1023};
	// set motor speed to 20%
	if (mtrHt->setVelocity(motors::right,velocity)) return -41;
	if (mtrHt->setVelocity(motors::left,velocity)) return -41;
	cout << "MotorCap POC: Mrun! Velo set to: " << velocity << endl;

	// set motor command to Advance
	if (mtrHt->runMotor(motors::right, mtrCmd::advance)) return -44;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance)) return -44;
	cout << "MotorCap POC: Mrun! cmd advance set" << endl;

	sleep(SLPTM );		// Wait amount sec to see the action.

	if (mtrHt->runMotor(motors::right, mtrCmd::release)) return -48;
	if (mtrHt->runMotor(motors::left, mtrCmd::release)) return -48;
	cout << "MotorCap POC: Mrun! cmd release set" << endl;

	sleep(1);

	if (mtrHt->runMotor(motors::right, mtrCmd::reverse)) return -44;
	if (mtrHt->runMotor(motors::left, mtrCmd::reverse)) return -44;
	cout << "MotorCap POC: Mrun! cmd reverse set" << endl;

	sleep(SLPTM );		// Wait amount sec to see the action.


	// set motor velocity to 0%
	velocity = 0;
	mtrHt->setVelocity(motors::right, velocity);
	mtrHt->setVelocity(motors::left, velocity);
	cout << "MotorCap POC: Mrun! Velocity set to: " << velocity << endl;
	sleep (1);			// Wait 1 sec to see the action.
	if (mtrHt->runMotor(motors::right, mtrCmd::release)) return -48;
	if (mtrHt->runMotor(motors::left, mtrCmd::release)) return -48;
	cout << "MotorCap POC: Mrun! cmd release set" << endl;

	return 0;
} //---[ testRun ]---

***/

/***
int testReverse(MotorCAP *mtrHt) {
	cout << endl << "MotorCap POC: Start testReverse!" << endl;
	// set motor velocity to ~25%%
	uint8_t velocity {25};
	mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
	mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;
	sleep (1);		// Wait amount sec to see the action.

	// set motor velocity to ~50%
	velocity = 50;
	mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
	mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
	cout << "MotorCap POC: testReverse! Velocity set to: " << unsigned(velocity) << endl;
	sleep (3);			// Wait 1 sec to see the action.

	// set motor velocity to ~75%
	velocity = 75;
	mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
	mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
	cout << "MotorCap POC: testReverse! Velocity set to: " << unsigned(velocity) << endl;
	sleep (3);			// Wait 1 sec to see the action.

	// set motor velocity to ~1000%
	velocity = 100;
	mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
	mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
	cout << "MotorCap POC: testReverse! Velocity set to: " << unsigned(velocity) << endl;
	sleep (3);			// Wait 1 sec to see the action.

	// set motor velocity to 0%
	velocity = 0;
	mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
	mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
	cout << "MotorCap POC: testReverse! Velocity set to: " << unsigned(velocity) << endl;

	sleep (1);			// Wait 1 sec to see the action.
	if (mtrHt->releaseMotor(motors::right)) return -47;
	if (mtrHt->releaseMotor(motors::left)) return -48;
	cout << "MotorCap POC: testReverse! cmd release set" << endl;

	return 0;
} //---[ testReverse ]---
***/

/***
int testAdvance(MotorCAP *mtrHt) {
	cout << "MotorCap POC: testAdvance Start!" << endl;

	// set motor speed to 0%
	uint8_t velocity {0};

	while (velocity < 100) {
		velocity += 20;
		mtrHt->runMotor(motors::right, mtrCmd::advance, velocity);
		mtrHt->runMotor(motors::left, mtrCmd::advance, velocity);
		cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;
		sleep (3);		// Wait amount sec to see the action.
	}


	// set motor velocity to 0%
	velocity = 0;
	if (mtrHt->runMotor(motors::right, mtrCmd::advance, velocity)) return -43;
	if (mtrHt->runMotor(motors::left, mtrCmd::advance, velocity)) return -44;
	cout << "MotorCap POC: testAdvance! cmd advance set with velo off: " << unsigned(velocity)  << endl;

	sleep (1);			// Wait 1 sec to see the action.
	if (mtrHt->releaseMotor(motors::right)) return -47;
	if (mtrHt->releaseMotor(motors::left)) return -48;
	cout << "MotorCap POC: testAdvance! cmd release set" << endl;

	return 0;
} //---[ testAdvance ]---
***/

/***
int testReverse(MotorCAP *mtrHt) {
	cout << endl << "MotorCap POC: Start testReverse!" << endl;
	// set motor velocity to ~25%%
	uint8_t velocity {0};

	while (velocity < 100) {
		velocity += 20;
		mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
		mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
		cout << "MotorCap POC: testReverse! cmd reverse set with velo off: " << unsigned(velocity)  << endl;
		sleep (3);		// Wait amount sec to see the action.
	}

	// set motor velocity to 0%
	velocity = 0;
	mtrHt->runMotor(motors::right, mtrCmd::reverse, velocity);
	mtrHt->runMotor(motors::left, mtrCmd::reverse, velocity);
	cout << "MotorCap POC: testReverse! Velocity set to: " << unsigned(velocity) << endl;

	sleep (1);			// Wait 1 sec to see the action.
	if (mtrHt->releaseMotor(motors::right)) return -47;
	if (mtrHt->releaseMotor(motors::left)) return -48;
	cout << "MotorCap POC: testReverse! cmd release set" << endl;

	return 0;
} //---[ testReverse ]---
***/



int testDiffAdvance(MotorCAP *mtrHt)
{
	cout << "MotorCap POC: testAdvance Start!" << endl;

	// set motor pcm to 0
	int16_t pcm {0};

	while (pcm < 4096) {
		pcm += 600;
		mtrHt->runMotor(motors::right, pcm);
		mtrHt->runMotor(motors::left, pcm);
		cout << "MotorCap POC: testDiffAdvance! cmd advance set with pcm of: " << pcm << endl;
		sleep (2);		// Wait amount sec to see the action.
	}


	// Stop motor
	if (mtrHt->stopMotor(motors::right)) return -43;
	if (mtrHt->stopMotor(motors::left)) return -44;
	cout << "MotorCap POC: testDiffAdvance! stop motor" << endl;
	sleep (1);			// Wait 1 sec to see the action.

	if (mtrHt->releaseMotor(motors::right)) return -47;
	if (mtrHt->releaseMotor(motors::left)) return -48;
	cout << "MotorCap POC: testDiffAdvance! release motor" << endl;

	return 0;
} //---[ testDiffAdvance ]---


int testDiffReverse(MotorCAP *mtrHt)
{
	cout << endl << "MotorCap POC: Start testDiffReverse!" << endl;
	// set motor pcm to 0
	int16_t pcm {0};

	while (pcm > -4096) {
		pcm -= 600;
		mtrHt->runMotor(motors::right, pcm);
		mtrHt->runMotor(motors::left, pcm);
		cout << "MotorCap POC: testReverse! cmd reverse set with velo off: " << pcm << endl;
		sleep (3);		// Wait amount sec to see the action.
	}

	// Stop motor
	if (mtrHt->stopMotor(motors::right)) return -43;
	if (mtrHt->stopMotor(motors::left)) return -44;
	cout << "MotorCap POC: testDiffAdvance! stop motor" << endl;
	sleep (1);			// Wait 1 sec to see the action.

	if (mtrHt->releaseMotor(motors::right)) return -47;
	if (mtrHt->releaseMotor(motors::left)) return -48;
	cout << "MotorCap POC: testReverse! cmd release set" << endl;

	return 0;
} //---[ testDiffReverse ]---


int main() {
	cout << "MotorCap POC: Test Concept V0.0.4!" << endl;

	// Initiate an instance of motorHat
	MotorCAP *motorCap = new MotorCAP(MOTORCAP_DEVID);

	cout << "MotorCap POC: MotorCAP Initiated!" << endl;

	// Run the tests

	cout << endl;
	if (testDiffReverse(motorCap )) return -50;

	cout << " Motors should be off!" << endl;
	sleep (5);		// Wait 5 sec to see the action.
	cout << " Continuing!" << endl;


	cout << endl;
	if (testDiffAdvance(motorCap )) return -49;



	/**
	cout << "MotorCap Motor Right" << endl;
	if (testMrun (motorHat, motors::right)) return -49;
	sleep (1);		// Wait 5 sec to see the action.
	cout << "MotorCap Motor Left" << endl;
	if (testMrun (motorHat, motors::left)) return -49;
	sleep (1);		// Wait 5 sec to see the action.

	cout << "MotorCap Motors Right and Left" << endl;
	if (testRun (motorHat ) ) return -49;

	cout << endl;
	if (testReverse (motorHat )) return -50;

	sleep (1);		// Wait 5 sec to see the action.

	cout << endl;
	if (testAdvance(motorHat )) return -49;
	**/

	motorCap->~MotorCAP();

	return 0;
} //---[ end main ]---


//===[ eof motorcap_poc_main.cpp ]===//
