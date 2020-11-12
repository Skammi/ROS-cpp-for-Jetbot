/*
 *	file :			jetbot_stats_class.cpp
 *	Project :		jetbot
 *	Created on :	10 Nov 2020
 *	Last edit :		11 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201110	initial version
 *
 *	Contains the JetbotStats Class
 *
 */

#include <jetbot_stats/jetbot_stats_class.h>

// Remove comment on below line, to get more info messages.
//#define INCREASE_ROS_INFO

// Jetbot ina219 address
#define INAJETBOT_BUSID		1
#define INAJETBOT_DEVID		0x41




#define PUBBUFFERSIZE	1	//buffer size of 1; could make larger, if expect network backups
#define SUBBUFFERSIZE	10	//buffer size of 10; could make larger, if expect network backups


//==================[ Public methods ]=======================


/*
 * Function	: Constructor
 * Purpose	: To initialize the hardware via Ina219 and publ;ishers, timers and variables
 * Input	: Ros node handle
 * Output	: No return code hardware and software are initialized
 */
JetbotStats::JetbotStats(ros::NodeHandle* nodehandle) : Ina219(INAJETBOT_BUSID, INAJETBOT_DEVID)
{
	nh_ = *nodehandle;
	ROS_INFO("JetbotStats: in class constructor");

	// Initialize the parameter variables
	this->getParameters();
	// Initialize the Publishers and Timers
	this->initializePublishers();
	this->initializeTimers();

	//initialize variables here, as needed
	statsPubMsg_.header.frame_id = "Jetbot Stats";
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
	nh_.param("/jetbot_stats/stats_pub_pulse_", statsPubPulse_, statsPubPulse_);

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

	statsPub_ = nh_.advertise<jetbot_msg::statsPub>("jetbot_stats_topic", PUBBUFFERSIZE);
	// test with: rostopic echo jetbot_stats_topic

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
			&JetbotStats::getPowerStatsCallback, this);

	// Stats publish timer
	timerPublishStats_ = nh_.createTimer(ros::Duration(statsPubPulse_),
			&JetbotStats::StatsPubCallback, this);

} //--[ end initializeTimers ]--//


/*
 * Function	: StatsPubCallback
 * Purpose	: To Publish the available stats
 * Input	: _timerEvent
 * Output	: No return code. Stats msg published
 */
void JetbotStats::StatsPubCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: StatsPubCallback activated.");
	#endif
	statsPubMsg_.header.seq++;								//increment the sequence counter
	statsPubMsg_.header.stamp =	_timerEvent.current_real;	//update the time stamp

	// Data is update in the Call back routines.

	statsPub_.publish(statsPubMsg_);
} //--[ end StatsPubCallback ]--//

/*
 * Function	: getPowerStatsCallback
 * Purpose	: To get the power stats from the ina219 and update the stats msg
 * Input	: _timerEvent
 * Output	: No return code, Stats msg updated
 */
void JetbotStats::getPowerStatsCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("JetbotStats: getPowerStatsCallback activated.");
	#endif
	statsPubMsg_.supply_voltage_v = Ina219::getBusVoltage();
	statsPubMsg_.supply_current_ma = Ina219::getCurrent();
} //--[ end initializeTimers ]--//


//------------------[ specific methods ]------------------------




//****[ eof jetbot_stats_class.cpp ]****//

