/*
 *	file :			jetbot_node.cpp
 *	Project :		jetbot
 *	Created on :	2 Oct 2020
 *	Last edit :		3 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.1.1	201102	Adapted shutdown to ensure the motors are stopped when programs exits.
 *	0.1.0	201023	Migrated to jeybot_node using motorcap instead of drivejetson
 *	0.0.1	201002	initial poc version adapted from myros_lib
 *
 *	To be started with a launch file, which has to set the jetbot specific parameters
 *	Start:
 *	roslaunch jetbot_node jetbot_core.launch
 *
 */

// Include the class header file
#include <jetbot_node/jetbot_class.h>

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
	// ROS set-ups:
	ros::init(argc, argv, "jetbot_node", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	// Override the default ros sigint handler.
	signal(SIGINT, mySigIntHandler);

	JetbotRos JetbotRos(&nh);		//instantiate a JetbotRos object

	float nap {10.0};				// Hz Default nap periode
	nh.param("/jetbot/nap_periode", nap, nap);		// get custom nap periode
	ros::Rate naptime(nap); 		// create a rate object of 10 Hz;

	ROS_INFO("jetbot_node: going into spin; let the call-backs do all the work");

	// Main program loop.
	while (noShutdownRequest) {
		ros::spinOnce();
		naptime.sleep();	// Sleep for the balance period
	}

	ROS_INFO("jetbot_node: Left main program loop.");

	// Clean up before we close.
	JetbotRos.brake();		// Stop the jetbot
	ros::shutdown();		// Shutdown this node.

    return 0;
} 

//****[ eof jetbot_node.cpp ]****//
