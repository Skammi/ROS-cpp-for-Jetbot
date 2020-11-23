/*
 *	file :			jetbot.cpp
 *	Project :		jetbot
 *	Created on :	2 Oct 2020
 *	Last edit :		21 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.1.4	201121	Adjusted the missed heartbeat message logging.
 *	0.1.3	201103	Added subscriber to mux/heartbeat.
 *	0.1.0	201023	Changed to use differential drive steering to accommodate twist message.
 *	0.0.6	201006	Added jetbot action SP service.
 *	0.0.5	201005	Added jetbot publisher.
 *	0.0.4	201005	Added jetbot state service.
 *	0.0.3	201004	Added jetbot specific messages and subscriber.
 *	0.0.2	201003	Made first adaption to include driveJetbot
 *	0.0.1	201002	initial poc version adapted from myros_lib
 *
 *	Contains the JetbotRos Class
 *
 */

#include <jetbot_node/jetbot_class.h>

// Remove comment on below line, to get more info messages.
//#define INCREASE_ROS_INFO

#define PUBBUFFERSIZE	1	// buffer size of 1; could make larger, if expect network backups
#define SUBBUFFERSIZE	10	// buffer size of 10; could make larger, if expect network backups


//==================[ Public methods ]=======================


/*
 * Function	: Constructor
 * Purpose	: To initialize the hardware via MotorCAP and subscribers and publishers
 * Input	: Ros node handle
 * Output	: No return code hardware and software are initialized
 */
JetbotRos::JetbotRos(ros::NodeHandle* nodehandle) : MotorCAP()
{
	nh_ = *nodehandle;
	ROS_INFO("jetbot: in class constructor of JetbotRos library");

	// Initialize the parameter variables
	this->getParameters();
	// Initialize the Subscribers, Timers
	this->initializeSubscribers();
	this->initializeTimers();

	//initialize variables here, as needed
	time_1ActionSrv = ros::Time::now();		// set the time the last request was received to now

} //--[ end constructor ]--//

JetbotRos::~JetbotRos() {
	// stop and release the motors
	JetbotRos::brake();
	this->~MotorCAP();
} //--[ end destructor ]--//


/*
 * Function	: brake
 * Purpose	: to set the motors velocity to 0 and release the motors.
 * Input	: None called by the destructor or from main
 * Output	: No return code
 */
void JetbotRos::brake(void)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: brake activated.");
	#endif

	MotorCAP::stopMotor(motors::right );
	MotorCAP::stopMotor(motors::left );

	MotorCAP::releaseMotor(motors::right );
	MotorCAP::releaseMotor(motors::left );

} //--[ end brake ]--//


//==================[ Private methods ]=======================


//------------------[ Initialize methods ]--------------------

/*
 * Function	: getParameters
 * Purpose	: To set variables according to the parameters
 * Input	: none
 * Output	: No return code, class parameter variables set.
 */
void JetbotRos::getParameters()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: getParameters activated.");
	#endif

	nh_.param("/jetbot/width", width_, width_);
	nh_.param("/jetbot/ratio", ratio_, ratio_);
	nh_.param("/jetbot/heartbeat_offset", heartbeatOffset_, heartbeatOffset_);
	nh_.param("/jetbot/max_missed_heartbeats", maxMissedHeartbeats_, maxMissedHeartbeats_);
	nh_.param("/twist_mux/heartbeat", muxHeartbeat_, muxHeartbeat_);

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: getParameters values width: %f | ratio: %f ." ,width_ ,ratio_ );
	#endif

} //--[ end getParameters ]--//


/*
 * Function	: initializeSubscribers
 * Purpose	: To set up subscribers
 * Input	: none
 * Output	: No return code, subscribers setup.
 */
void JetbotRos::initializeSubscribers()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: Initializing Subscribers");
	#endif

	cmd_vel_sub_ = nh_.subscribe("mux/cmd_vel", SUBBUFFERSIZE, &JetbotRos::twistCallback, this);
	// test with: rostopic pub -r 2 mux/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
	muxHeartbeat_sub_ = nh_.subscribe("mux/heartbeat", SUBBUFFERSIZE, &JetbotRos::muxHeartbeatCallback, this);
	// test with: rostopic pub -r 2 mux/heartbeat std_msgs::Bool '{data: true}'

} //--[ end initializeSubscribers ]--//


/*
 * Function	: initializeTimers
 * Purpose	: To set up timers
 * Input	: none
 * Output	: No return code, timers setup.
 */
void JetbotRos::initializeTimers() {
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: Initializing Timers");
	#endif

	muxHeartbeatCheck_timer_ = nh_.createTimer(ros::Duration(muxHeartbeat_),
			&JetbotRos::muxHeartbeatTimerCallback, this);

}  //--[ end initializeTimers ]--//



//------------------[ callback methods ]------------------------


/*
 * Function	: twistCallback
 * Purpose	: To handle the new twist message .
 * Input	: geometry_msgs/Twist message
 * Output	: no return code. Twist message handled
 */
void JetbotRos::twistCallback(const geometry_msgs::Twist& _twist)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: twistCallback activated");
	#endif

	if ( not differentialDrive(_twist )) ROS_ERROR ("jetbot: twistCallback: error when sending twist to motors");

} //--[ end twistCallback ]--//


/*
 * Function	: muxHeartbeatCallback
 * Purpose	: To reset the heart-beat timer and missed heart-beats
 * Input	: std_msgs::Bool
 * Output	: no return code.
 */
void JetbotRos::muxHeartbeatCallback(const std_msgs::Bool& _heartbeat)
{
	// if heart-beat restart timer
	if (_heartbeat.data) {
		T_1muxHeartbeat_ = ros::Time::now().toSec();
		missedHeartBeats_ = 0;			// Reset the missed Heart-beat counter
		if (logHeartBeatError) {
			ROS_ERROR("jetbot: Heart-beat restored.");
			logHeartBeatError = false;	// Reset the log heart-beat flag
		}
	}
} //--[ end muxHeartbeatCallback ]--//


/*
 * Function	: muxHeartbeatTimerCallback
 * Purpose	: To handle missed beats when needed
 * Input	: ros::TimerEvent
 * Output	: no return code.
 */
void JetbotRos::muxHeartbeatTimerCallback(const ros::TimerEvent& _timerEvent)
{
	// Calculate the time past
	double deltaT = _timerEvent.current_real.toSec() - T_1muxHeartbeat_;

	// are heart-beats missed ?
	if (deltaT > heartbeatOffset_) {
		missedHeartBeats_ += 1;
		#ifdef INCREASE_ROS_INFO
		ROS_INFO("jetbot: Missed Heart-beats %d .", missedHeartBeats_);
		#endif
	}

	// Are there more missed
	if (missedHeartBeats_ > maxMissedHeartbeats_) {
		this->brake();			// Stop the jetbot
		if (not logHeartBeatError) {
			ROS_ERROR("jetbot: No Heart-beat stopping motors.");
			logHeartBeatError = true;
		}


	}
} //--[ end muxHeartbeatTimerCallback ]--//


//------------------[ specific methods ]------------------------

/*
 * Function	: differentialDrive
 * Purpose	: To steer the motors with the new value
 * Input	: twist message Normalized -1 <= X <= 1
 * Output	: True if successful.
 */
bool JetbotRos::differentialDrive(geometry_msgs::Twist cmd_vel_)
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: differentialDrive activated.");
	#endif

	float linearX = cmd_vel_.linear.x;
	float angularZ = cmd_vel_.angular.z;

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: differentialDrive Linear X = %f ", linearX);
	ROS_INFO("jetbot: differentialDrive Angular Z = %f ", angularZ);
	#endif

	// Split the linear and angular velocity out;
	float veloRight = ( 1.0 * linearX ) + (angularZ * width_ /2);
	float veloLeft = ( 1.0 * linearX ) - (angularZ * width_ /2);

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: differentialDrive velo Right = %f ", veloRight);
	ROS_INFO("jetbot: differentialDrive velo Left = %f ", veloLeft);
	#endif

	// Convert velocity to raw PWM 0x0fff max
	int16_t veloRight_PWM = int16_t (veloRight * ratio_);
	int16_t veloLeft_PWM = int16_t (veloLeft * ratio_);

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("jetbot: differentialDrive velo Right pcm = %d ", veloRight_PWM);
	ROS_INFO("jetbot: differentialDrive velo Left pcm = %d ", veloLeft_PWM);
	#endif

	// Steer the motors exit when failed
	if (MotorCAP::runMotor (motors::right, veloRight_PWM)) return false;
	if (MotorCAP::runMotor (motors::left, veloLeft_PWM)) return false;

	return true;
} //--[ end differentialDrive ]--//




//****[ eof jetbot.cpp ]****//
