/*
 *	file :			jetbot_stats_class.cpp
 *	Project :		jetbot
 *	Created on :	10 Nov 2020
 *	Last edit :		23 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.3	201123	Change rpm to one request for both wheels
 *	0.0.2	201118	Added RPM
 *	0.0.1	201110	initial version
 *
 *	Contains the JetbotStats Class declaration
 *	Containing:
 *		Power publishing
 *		Rpm Pub
 *
 *	Note: RPM Publisher.
 *	The RPM Arduino-Nano calculates the RPM every 0.5 second.
 *	A sample time of 0.25 sec., ensures the sample value age is less then 0.5 second.
 *
 *	Timing parameters are default set to 1 second. Practical values are obtained from
 *	the parameter server provided by: jetbot_stats_parms.yaml
 */

#include <jetbot_stats/jetbot_stats_class.h>

// Remove comment on below line, to get more info messages.
//#define INCREASE_ROS_INFO

// Jetbot ina219 address
#define JETBOT_BUSID		1
#define INAJETBOT_DEVID		0x41
#define RPMSENSOR_DEVID		0x11




#define PUBBUFFERSIZE	1	//buffer size of 1; could make larger, if expect network backups
#define SUBBUFFERSIZE	10	//buffer size of 10; could make larger, if expect network backups


//==================[ Public methods ]=======================


/*
 * Function	: Constructor
 * Purpose	: To initialize the hardware via Ina219 and publ;ishers, timers and variables
 * Input	: Ros node handle
 * Output	: No return code hardware and software are initialized
 */
JetbotStats::JetbotStats(ros::NodeHandle* nodehandle) :
	Ina219(JETBOT_BUSID, INAJETBOT_DEVID),
	RpmSensor(RPMSENSOR_DEVID)
{
	nh_ = *nodehandle;
	ROS_INFO("JetbotStats: in class constructor");

	// Initialize the parameter variables
	this->getParameters();
	// Initialize the Publishers and Timers
	this->initializePublishers();
	this->initializeTimers();

	//initialize variables here, as needed
	powerPubMsg_.header.frame_id = "Jetbot power";
	rpmPubMsg_.header.frame_id = "Jetbot RPM";
} //--[ end constructor ]--//

JetbotStats::~JetbotStats()
{
	// Nothing special for now
} //--[ end destructor ]--//


//==================[ Private methods ]=======================


//------------------[ Initialize methods ]--------------------

/*
 * Function	: getParameters
 * Purpose	: To set variables according to the parameters
 * Input	: none
 * Output	: No return code, class parameter variables set.
 */
void JetbotStats::getParameters()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: getParameters activated.");
	#endif

	nh_.param("/jetbot_stats/power_get_pulse_", powerGetPulse_, powerGetPulse_);
	nh_.param("/jetbot_stats/rpm_get_pulse_", rpmGetPulse_, rpmGetPulse_);
	nh_.param("/jetbot_stats/power_pub_pulse_", powerPubPulse_, powerPubPulse_);
	nh_.param("/jetbot_stats/rpm_pub_pulse_", powerPubPulse_, rpmPubPulse_);

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: getParameters values width: %f | ratio: %f ." ,width_ ,ratio_ );
	#endif

} //--[ end getParameters ]--//


/*
 * Function	: initializePublishers
 * Purpose	: To set up publishers
 * Input	: none
 * Output	: No return code, publishers setup.
 */
void JetbotStats::initializePublishers()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: Initializing Publishers");
	#endif

	powerPub_ = nh_.advertise<jetbot_msg::powerPub>("jetbot_power_topic", PUBBUFFERSIZE);
	// test with: rostopic echo jetbot_power_topic

	rpmPub_ = nh_.advertise<jetbot_msg::rpmPub>("jetbot_rpm_topic", PUBBUFFERSIZE);
	// test with: rostopic echo jetbot_rpm_topic

}  //--[ end initializePublishers ]--//

/*
 * Function	: initializeTimers
 * Purpose	: To set up timers
 * Input	: none
 * Output	: No return code, timers setup.
 */
void JetbotStats::initializeTimers()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: Initializing Timers");
	#endif

	// power stats read timer
	timerReadIna219_ = nh_.createTimer(ros::Duration(powerGetPulse_),
			&JetbotStats::getPowerCallback, this);

	// power stats read timer
	timerReadRpmSensor_ = nh_.createTimer(ros::Duration(rpmGetPulse_),
			&JetbotStats::getRpmCallback, this);

	// Stats publish timer
	timerPublishPower_ = nh_.createTimer(ros::Duration(powerPubPulse_),
			&JetbotStats::powerPubCallback, this);

	// Stats publish timer
	timerPublishRpm_ = nh_.createTimer(ros::Duration(rpmPubPulse_),
			&JetbotStats::rpmPubCallback, this);

} //--[ end initializeTimers ]--//


/*
 * Function	: powerPubCallback
 * Purpose	: To Publish the available stats
 * Input	: _timerEvent
 * Output	: No return code. power msg published
 */
void JetbotStats::powerPubCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: powerPubCallback activated.");
	#endif
	powerPubMsg_.header.seq++;								// increment the sequence counter
	powerPubMsg_.header.stamp =	_timerEvent.current_real;	// update the time stamp

	// Data is update in the Call back routines.
	powerPub_.publish(powerPubMsg_);
} //--[ end powerPubCallback ]--//


/*
 * Function	: rpmPubCallback
 * Purpose	: To Publish the available stats
 * Input	: _timerEvent
 * Output	: No return code. rpm msg published
 */
void JetbotStats::rpmPubCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: rpmPubCallback activated.");
	#endif
	rpmPubMsg_.header.seq++;								// increment the sequence counter
	rpmPubMsg_.header.stamp =	_timerEvent.current_real;	// update the time stamp

	// Data is update in the Call back routines.
	rpmPub_.publish(rpmPubMsg_);
} //--[ end rpmPubCallback ]--//


/*
 * Function	: getPowerCallback
 * Purpose	: To get the power values from the ina219 and update the stats msg
 * Input	: _timerEvent
 * Output	: No return code, power msg updated
 */
void JetbotStats::getPowerCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: getPowerStatsCallback activated.");
	#endif
	powerPubMsg_.supply_voltage_v = Ina219::getBusVoltage();
	powerPubMsg_.supply_current_ma = Ina219::getCurrent();
} //--[ end getPowerCallback ]--//


/*
 * Function	: getRpmCallback
 * Purpose	: To get the  rpm values from the rpm-sensor and update the stats msg
 * Input	: _timerEvent
 * Output	: No return code, RPM msg updated
 */
void JetbotStats::getRpmCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: getRpmStatsCallback activated.");
	#endif
	uint16_t *sensors = new uint16_t[2];			// create room for 2 16bit int
	sensors = RpmSensor::readRpmRegisters(rpmRegister::rpm_ab);
	//rpmPubMsg_.rpm_right_wheel = RpmSensor::readRpmRegister(rpmRegister::rpm_a);
	//rpmPubMsg_.rpm_left_wheel = RpmSensor::readRpmRegister(rpmRegister::rpm_b);
	rpmPubMsg_.rpm_right_wheel = sensors[0];
	rpmPubMsg_.rpm_left_wheel = sensors[1];

} //--[ end getRpmCallback ]--//




//------------------[ specific methods ]------------------------




//****[ eof jetbot_stats_class.cpp ]****//

