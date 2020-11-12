/*
 *	file :			twist_mux_class.h
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
 *	Contains the TwistMux Class definition
 *
 */

#ifndef TWIST_MUX_H_
#define TWIST_MUX_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TwistMux
{
public:
	TwistMux(ros::NodeHandle* _nodehandle);
	~TwistMux(void);
	void sendStop(void);

private:
	ros::NodeHandle nh_;

	// Parameters
	double blockAutoTime_	{5.0};		// block auto time in sec
	double heartbeat_		{1.0};		// heart beat every sec

	// Memory
	double teleopT_1Sec_;

	ros::Publisher cmd_velPub_;
	ros::Publisher muxHeartBeatPub_;
	ros::Subscriber teleopSub_;
	ros::Subscriber autoSub_;
	ros::Timer heartbeatTimer_;		// The heart beat timer identifier.

	// methods initializers:
	void getParameters(void);
	void initializeSubscribers(void);
	void initializePublishers(void);
	void initializeTimers(void);

	// Callback methods
	void teleopCallback(const geometry_msgs::Twist::ConstPtr& _teleopTwist);
	void autoCallback(const geometry_msgs::Twist::ConstPtr& _autoTwist);
	void heartbeatCallback(const ros::TimerEvent& _timerEvent);
};

#endif //=== TWIST_MUX_H_ ===//

