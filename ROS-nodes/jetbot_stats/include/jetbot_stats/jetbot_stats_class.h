/*
 *	file :			jetbot_stats_class.h
 *	Project :		jetbot
 *	Created on :	10 Nov 2020
 *	Last edit :		11 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201110	initial version
 *
 *	Contains the JetbotStats Class decleration
 *
 */

#ifndef JETBOT_STATS_CLASS_H_
#define JETBOT_STATS_CLASS_H_

// some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// The ina219 library
#include <ina219.h>

// The ROS library
#include <ros/ros.h>

//message types used.
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <jetbot_msg/statsPub.h>				// Get the power Publisher message details

// the Ina219 class is in the embed(ded) name space.
using namespace embed;

// Class declaration
class JetbotStats : protected Ina219
{
	public:
		JetbotStats(ros::NodeHandle* nodehandle);
		~JetbotStats(void);
	private:
		// Class global variables
		ros::NodeHandle nh_;
		ros::Publisher statsPub_;
		ros::Timer timerReadIna219_;
		ros::Timer timerPublishStats_;

		jetbot_msg::statsPub statsPubMsg_;	// The state publisher message

		// Parameter variables
		float powerGetPulse_ {1.0};			// power publisher every second
		float statsPubPulse_ {1.0};			// power publisher every second

		// methods initializers:
		void getParameters(void);
		void initializePublishers(void);
		void initializeTimers(void);

		// Callback methods
		void StatsPubCallback(const ros::TimerEvent& _timerEvent);
		void getPowerStatsCallback(const ros::TimerEvent& _timerEvent);
};

#endif //=== JETBOT_STATS_CLASS_H_ ===//
