/*
 *	file :			jetbot_class.h
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
 *	0.0.4	201005	Added jetbot action PV service.
 *	0.0.3	201004	Added jetbot specific messages and subscriber.
 *	0.0.2	201003	Made first adaption to include driveJetbot
 *	0.0.1	201002	initial poc version adapted from myros_lib
 *
 *	Contains the JetbotRos Class
 *
 */

#ifndef JETBOT_CLASS_H_
#define JETBOT_CLASS_H_

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// The motor driver library
#include <motorcap.h>

// The ROS library
#include <ros/ros.h>

//message types used.
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>			// uses the "Trigger.srv" message defined in ROS
#include <geometry_msgs/Twist.h>		// used for the command message
#include <jetbot_msg/actionPub.h>		// used for the actionPub publisher
#include <jetbot_msg/statePub.h>

// the JetBot drive control is in the embed(ded) name space.
using namespace embed;

// Class declaration
class JetbotRos : public MotorCAP {
	public:
		JetbotRos(ros::NodeHandle* nodehandle);
		~JetbotRos(void);
		void brake(void);
	private:
		// Class global variables
		ros::NodeHandle nh_;
		ros::Subscriber cmd_vel_sub_;
		ros::Subscriber muxHeartbeat_sub_;
		ros::Timer muxHeartbeatCheck_timer_;	// The heart beat timer identifier.

		// memory
		ros::Time time_1ActionSrv;				// To store time last request was received
		ros::Time time_1TwistCallback;			// To store time last subscription was received

		double T_1muxHeartbeat_;				// To store time last heart-beat was received
		int	missedHeartBeats_ {0};				// number of heart-beats missed.
		bool logHeartBeatError {false};			// To log Heart-beat error only once.

		// Parameter variables
		double width_ {0};						// Width of the jetbot in meters
		double ratio_ {0};						// ratio Meters/Sec to percentage
		double heartbeatOffset_ {2.0};			// Time in which a heart-beat must be received
		double muxHeartbeat_ {1.0};				// mux heart beat every sec
		int maxMissedHeartbeats_ {2};			// Maximum number of heart-beats that can be missed

		// methods initializers:
		void getParameters(void);
		void initializeSubscribers(void);
		void initializeTimers(void);

		// Callback methods
		void twistCallback(const geometry_msgs::Twist& _twist);
		void muxHeartbeatCallback(const std_msgs::Bool& _heartbeat);
		void muxHeartbeatTimerCallback(const ros::TimerEvent& _timerEvent);

		// specific methods
		bool differentialDrive(geometry_msgs::Twist cmd_vel_);
};

#endif //=== JETBOT_CLASS_H_ ===//
