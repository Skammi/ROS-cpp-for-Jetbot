/*
 *	file :			jetbot_stats_class.h
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

#ifndef JETBOT_STATS_CLASS_H_
#define JETBOT_STATS_CLASS_H_

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// Used libraries
#include <ina219.h>		//Ina library for power
#include <rpmsensor.h>	// rpmsensor library for rpm's of wheels

// The ROS library
#include <ros/ros.h>

//message types used.
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <jetbot_msg/powerPub.h>				// Get the power Publisher message details
#include <jetbot_msg/rpmPub.h>					// Get the RPM Publisher message details

// the Ina219 class is in the embed(ded) name space.
using namespace embed;

// Class declaration
class JetbotStats : protected Ina219, RpmSensor
{
	public:
		JetbotStats(ros::NodeHandle* nodehandle);
		~JetbotStats(void);
	private:
		// Class global variables
		ros::NodeHandle nh_;
		ros::Publisher powerPub_;
		ros::Publisher rpmPub_;
		ros::Timer timerReadIna219_;
		ros::Timer timerReadRpmSensor_;
		ros::Timer timerPublishRpm_;
		ros::Timer timerPublishPower_;

		jetbot_msg::powerPub powerPubMsg_;	// The state publisher message
		jetbot_msg::rpmPub rpmPubMsg_;	// The state publisher message

		// Parameter variables
		float powerGetPulse_ {1.0};			// power publisher every second
		float rpmGetPulse_ {1.0};			// power publisher every second
		float powerPubPulse_ {1.0};			// power publisher every second
		float rpmPubPulse_ {1.0};			// power publisher every second

		// methods initializers:
		void getParameters(void);
		void initializePublishers(void);
		void initializeTimers(void);

		// Callback methods
		void powerPubCallback(const ros::TimerEvent& _timerEvent);
		void rpmPubCallback(const ros::TimerEvent& _timerEvent);
		void getPowerCallback(const ros::TimerEvent& _timerEvent);
		void getRpmCallback(const ros::TimerEvent& _timerEvent);
};

#endif //=== JETBOT_STATS_CLASS_H_ ===//

