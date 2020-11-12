/*
 *	file :			twist_mux_class.cpp
 *	Project :		jetbot
 *	Created on :	2 Nov 2020
 *	Last edit :		11 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.3	201103	Changed heartbeat publisher trigger to timer
 *	0.0.2	201103	Added heartbeat publisher
 *	0.0.1	201102	Initial poc version adapted from ROS tutorial
 *
 *	Contains the twistMux Class
 *	Purpose:
 *
 */

#include <twist_mux/twist_mux_class.h>

// Remove comment on below line, to get more info messages.
//#define INCREASE_ROS_INFO

#define PUBBUFFERSIZE	1	//buffer size of 1; could make larger, if expect network backups
#define SUBBUFFERSIZE	10	//buffer size of 10; could make larger, if expect network backups

#define constrain(input,low,high) ((input)<(low)?(low):((input)>(high)?(high):(input)))



//==================[ Public methods ]=======================

/*
 * Function	: Constructor
 * Purpose	: To initialize subscribers and publishers
 * Input	: Ros node handle
 * Output	: No return code hardware and software are initialized
 */
TwistMux::TwistMux(ros::NodeHandle* _nodehandle)
{
	nh_ = *_nodehandle;
	TwistMux::getParameters();

	// Initialize the Subscribers, Publisher and Timers
	this->initializeSubscribers();
	this->initializePublishers();
	this->initializeTimers();
} //--[ end constructor ]--//

TwistMux::~TwistMux()
{
	// Send stop value out as we no longer control
	TwistMux::sendStop();
} //--[ end destructor ]--//

/*
 * Function	: sendStop
 * Purpose	: To send a zero twist command
 * Input	: none
 * Output	: No return code,
 */
void TwistMux::sendStop()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("TwistMux: sendStop activated.");
	#endif

	geometry_msgs::Twist twist;
	twist.angular.z = 0.0;
	twist.linear.x = 0.0;
	cmd_velPub_.publish(twist);		// Publish X = 0 and Z = 0
	//cmd_pub_.publish(cmd);

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("Joy2Twist: sendStop completed.");
	#endif
} //--[ end sendStop ]--//



//==================[ Private methods ]=======================


//------------------[ Initialize methods ]--------------------


/*
 * Function	: getParameters
 * Purpose	: To set variables according to the parameters
 * Input	: none
 * Output	: No return code, class parameter variables set.
 */
void TwistMux::getParameters()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("TwistMux: getParameters activated.");
	#endif

	// Get the definitions
	nh_.param("/twist_mux/block_auto_time", blockAutoTime_, blockAutoTime_);
	nh_.param("/twist_mux/heartbeat", heartbeat_, heartbeat_);


	#ifdef INCREASE_ROS_INFO
	ROS_INFO("Joy2Twist: getParameters completed.");
	#endif
} //--[ end getParameters ]--//


/*
 * Function	: initializeSubscribers
 * Purpose	: To set up subscribers
 * Input	: none
 * Output	: No return code, subscribers setup.
 */
void TwistMux::initializeSubscribers()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("TwistMux: Initializing Subscribers");
	#endif
	teleopSub_ = nh_.subscribe("teleop/cmd_vel", SUBBUFFERSIZE, &TwistMux::teleopCallback, this);
	autoSub_ = nh_.subscribe("auto/cmd_vel", SUBBUFFERSIZE, &TwistMux::autoCallback, this);
} //--[ end initializeSubscribers ]--//


/*
 * Function	: initializePublishers
 * Purpose	: To set up publishers
 * Input	: none
 * Output	: No return code, subscribers setup.
 */
void TwistMux::initializePublishers() {
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("TwistMux: Initializing Publishers");
	#endif
	muxHeartBeatPub_ = nh_.advertise<std_msgs::Bool>("mux/heartbeat", PUBBUFFERSIZE);
	cmd_velPub_ = nh_.advertise<geometry_msgs::Twist>("mux/cmd_vel", PUBBUFFERSIZE);
	// test with: rostopic echo mux/cmd_vel
} //--[ end initializePublishers ]--//


/*
 * Function	: initializeTimers
 * Purpose	: To set up timers
 * Input	: none
 * Output	: No return code, subscribers setup.
 */
void TwistMux::initializeTimers()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("TwistMux: Initializing Timers");
	#endif
	heartbeatTimer_ = nh_.createTimer(ros::Duration(1.0),
			&TwistMux::heartbeatCallback, this);
} //--[ end initializeTimers ]--//


//------------------[ callback methods ]------------------------

/*
 * Function	: teleopCallback
 * Purpose	: To handle the new joy stick message.
 * Input	: geometry_msgs::Twist
 * Output	: no return code.
 */
void TwistMux::teleopCallback(const geometry_msgs::Twist::ConstPtr& _teleopTwist)
{
	teleopT_1Sec_ = ros::Time::now().toSec();		// update time last teleop received
	cmd_velPub_.publish(_teleopTwist);				// publish teleop cmd_vel
} //--[ end teleopCallback ]--//


/*
 * Function	: autoCallback
 * Purpose	: To handle the new message.
 * Input	: geometry_msgs::Twist
 * Output	: no return code.
 */
void TwistMux::autoCallback(const geometry_msgs::Twist::ConstPtr& _autoTwist)
{
	// calculate the time past since last telop message
	double deltaT = ros::Time::now().toSec() - teleopT_1Sec_;
	if (deltaT >= blockAutoTime_)
    	cmd_velPub_.publish(_autoTwist);			// publish auto cmd_vel
} //--[ end autoCallback ]--//


/*
 * Function	: heartbeatCallback
 * Purpose	: To publish a new heart-beat.
 * Input	: ros::TimerEvent
 * Output	: no return code. hearty-beat send out.
 */
void TwistMux::heartbeatCallback(const ros::TimerEvent& _timerEvent)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("TwistMux: heartbeatCallback activated.");
	#endif
	std_msgs::Bool heartbeat;
	heartbeat.data = true;
    muxHeartBeatPub_.publish(heartbeat);			// publish that we are still alive
} //--[ end heartbeatCallback ]--//


//------------------[ specific methods ]------------------------




//****[ eof twist_mux_class.cpp ]****//

