/*
 *	file :			joy_twist_class.cpp
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
 *	Contains the Joy2Twist Class
 *	Purpose:
 *	Convert the Digital joy message to a twist message, Where the values are default restricted:
 *		-1 <= x <= 1
 *	These values can be changed using parameters.
 *
 */

#include <teleop_joy_twist/joy_twist_class.h>



// Remove comment on below line, to get more info messages.
//#define INCREASE_ROS_INFO

#define constrain(input,low,high) ((input)<(low)?(low):((input)>(high)?(high):(input)))


/*
 * Function	: Constructor
 * Purpose	: To initialize subscribers and publishers
 * Input	: Ros node handle
 * Output	: No return code hardware and software are initialized
 */
Joy2Twist::Joy2Twist(ros::NodeHandle* _nodehandle)
{
	nh_ = *_nodehandle;
	Joy2Twist::getParameters();

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("teleop/cmd_vel", 1);
	//ToDo: add a command publisher which will publish buttons pressed
	//cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("teleop/cmd_but", 1);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy2Twist::joyCallback, this);
} //--[ end constructor ]--//

Joy2Twist::~Joy2Twist()
{
	// Send stop value out as we no longer control
	Joy2Twist::sendStop();
} //--[ end destructor ]--//


/*
 * Function	: joyCallback
 * Purpose	: To handle the new joy message.
 * Input	: sensor_msgs/Joy message
 * Output	: no return code in subscriber functions.
 */
void Joy2Twist::joyCallback(const sensor_msgs::Joy::ConstPtr& _joy)
{
	geometry_msgs::Twist twist;
	//new messagetype cmd;

	// First check the stop button and return when completed
	if (_joy->buttons[indexStop_] == 1)
	{
		Joy2Twist::sendStop();
		return;							// then return
	}

	// Handle the Angular value
	cmdVelAng_ = cmdVelAng_ + (_joy->axes[indexAngular_] * stepAngular_);
	cmdVelAng_ = constrain (cmdVelAng_, minAngular_, maxAngular_);
	twist.angular.z = a_scale_ * cmdVelAng_;

	// Handle the Linear value
	cmdVelLin_ = cmdVelLin_ + (_joy->axes[indexLinear_] * stepLinear_);
	cmdVelLin_ = constrain (cmdVelLin_, minLinear_, maxLinear_);
	twist.linear.x = l_scale_ * cmdVelLin_;

	vel_pub_.publish(twist);
	//cmd_pub_.publish(cmd);
} //--[ end joyCallback ]--//


/*
 * Function	: getParameters
 * Purpose	: To set variables according to the parameters
 * Input	: none
 * Output	: No return code, class parameter variables set.
 */
void Joy2Twist::getParameters()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("Joy2Twist: getParameters activated.");
	#endif

	// Get the game controller definitions
	nh_.param("/gamepcps3/axis_linear", indexLinear_, indexLinear_);
	nh_.param("/gamepcps3/axis_angular", indexAngular_, indexAngular_);
	nh_.param("/gamepcps3/X_button", indexStop_, indexStop_);

	// Get the twist definitions
	nh_.param("/twist_definition/step_linear", stepLinear_, stepLinear_);
	nh_.param("/twist_definition/min_linear", minLinear_, minLinear_);
	nh_.param("/twist_definition/max_linear", maxLinear_, maxLinear_);
	nh_.param("/twist_definition/step_angular", stepAngular_, stepAngular_);
	nh_.param("/twist_definition/min_angular", minAngular_, minAngular_);
	nh_.param("/twist_definition/max_angular", maxAngular_, maxAngular_);


	#ifdef INCREASE_ROS_INFO
	ROS_INFO("Joy2Twist: getParameters values axLin: %d | axAng: %d ." ,indexLinear_ ,indexAngular_ );
	#endif
} //--[ end getParameters ]--//


/*
 * Function	: sendStop
 * Purpose	: To send a zero twist command
 * Input	: none
 * Output	: No return code,
 */
void Joy2Twist::sendStop()
{
	#ifdef INCREASE_ROS_INFO
	ROS_INFO("Joy2Twist: sendStop activated.");
	#endif

	geometry_msgs::Twist twist;
	cmdVelAng_ = 0.0;
	twist.angular.z = cmdVelAng_;
	cmdVelLin_ = 0.0;
	twist.linear.x = cmdVelLin_;
	vel_pub_.publish(twist);		// Publish X = 0 and Z = 0
	//cmd_pub_.publish(cmd);

	#ifdef INCREASE_ROS_INFO
	ROS_INFO("Joy2Twist: sendStop completed.");
	#endif
} //--[ end sendStop ]--//



//****[ eof joy_twist_class.cpp ]****//

