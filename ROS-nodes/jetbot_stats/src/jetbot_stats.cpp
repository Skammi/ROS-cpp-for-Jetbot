/*
 *	file :			jetbot_stats.cpp
 *	Project :		jetbot
 *	Created on :	10 Nov 2020
 *	Last edit :		10 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201110	initial version
 *
 *	Needs to be started with a launch file, which has to set the jetbot_stat specific parameters
 *	Start:
 *	roslaunch jetbot_stats jetbot_stats.launch
 *
 */

// Include the class header file
#include <jetbot_stats/jetbot_stats_class.h>

int main(int argc, char** argv) 
{
	// ROS set-ups:
	ros::init(argc, argv, "jetbot_stats");
	ros::NodeHandle nh;

	ROS_INFO("jetbot_stats: instantiating an object of type JetbotStats");
	JetbotStats jbs(&nh);	// instantiate a JetbotStats object

	ROS_INFO("jetbot_stats: Going to spin.");
	ros::spin();			// Let the call-backs do the work

    return 0;
} 

//****[ eof jetbot_stats.cpp ]****//
