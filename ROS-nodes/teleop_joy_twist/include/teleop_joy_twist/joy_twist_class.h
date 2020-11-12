/*
 *	file :			joy_twist_class.h
 *	Project :		jetbot
 *	Created on :	30 Oct 2020
 *	Last edit :		11 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.2	201101	Added parameters and stop button
 *	0.0.1	201030	initial poc version adapted from ROS tutorial
 *
 *	Contains the TwistMux Class definition
 *
 */

#ifndef JOY_TWIST_H_
#define JOY_TWIST_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class Joy2Twist
{
public:
	Joy2Twist(ros::NodeHandle* _nodehandle);
	~Joy2Twist(void);
	void sendStop(void);

private:
	void getParameters(void);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& _joy);


	ros::NodeHandle nh_;

	// Parameters
	int indexAngular_		{0};
	int indexLinear_		{1};
	int indexStop_			{3};

	double stepLinear_		{0.2};
	double minLinear_		{-1.0};
	double maxLinear_		{1.0};
	double stepAngular_		{0.2};
	double minAngular_		{-1.0};
	double maxAngular_		{1.0};

	// Memory
	int stopButton_			{0};

	float cmdVelLin_		{0.0};
	float cmdVelAng_		{0.0};
	double T_1CmdVelLin_	{0.0};
	double T_1CmdVelAng_	{0.0};


	double l_scale_ {1.0};
	double a_scale_ {1.0};

	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

};

#endif //=== JOY_TWIST_H_ ===//

