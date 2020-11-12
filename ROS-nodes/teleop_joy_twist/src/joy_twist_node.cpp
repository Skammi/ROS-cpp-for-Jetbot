/*
 *	file :			joy_twist_node.cpp
 *	Project :		jetbot
 *	Created on :	30 Oct 2020
 *	Last edit :		11 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201030	initial poc version adapted from ROS tutorial
 *
 *	Start:
 *	roslaunch teleop_joy_twist joy_twist.launch
 */

// Include the class header file
#include <teleop_joy_twist/joy_twist_class.h>

//+++[ Code block to shutdown in an orderly fashion.
#include <signal.h>

sig_atomic_t volatile noShutdownRequest = true;	// Signal-safe flag for whether shutdown is requested

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
	noShutdownRequest = false;		// this will break the main program loop.
} //--[ end mySigIntHandler ]--//
//+++[ END code block to shutdown in an orderly fashion.


int main(int argc, char** argv)
{
	// Setup ROS
	ros::init(argc, argv, "joy_twist_node", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh; 		// create the node handle to pass to the class constructor

	// Override the default ros sigint handler.
	signal(SIGINT, mySigIntHandler);

	Joy2Twist j2T (&nh);			// create a Joy2Twist object

	float nap {10.0};				// Hz Default nap periode
	nh.param("/joy_twist/nap_periode", nap, nap);		// get custom nap periode
	ros::Rate naptime(nap); 		// create a rate object of 10 Hz;

	ROS_INFO("joy_twist_node: going into spin; let the call-backs do all the work");

	// Main program loop.
	while (noShutdownRequest) {
		ros::spinOnce();
		naptime.sleep(); //sleep for the balance of the desired period to achieve the specified loop frequency
	}

	ROS_INFO("joy_twist_node: Left main program loop.");

	// Clean up before we close.
	j2T.sendStop(); 				// send a x=0 z=0 cmd_vel
	ros::shutdown();				// Shutdown this node.

	return 0;
} //--[ end main ]--//

//****[ eof joy_twist_node.cpp ]****//
