/*
 *	file :			ina219_poc_main.h
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


#ifndef INA219_POC_H_
#define INA219_POC_H_



#include <stdio.h>
#include <unistd.h>
#include <iostream>		// For cout << etc... << endl
#include <math.h>
#include <iomanip>		// For cli parameter conversion

#include <ina219.h>

#define SLPTM		2	// sec sleep time

#define INAJETBOT_BUSID		1
#define INAJETBOT_DEVID		0x41

using namespace std;
using namespace embed;

#endif /* INA219_POC_H_ */

