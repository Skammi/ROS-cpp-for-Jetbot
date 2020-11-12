/*
 *	file :			twist_mux_node.cpp
 *	Project :		jetbot
 *	Created on :	2 Nov 2020
 *	Last edit :		6 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.3	201103	Changed heartbeat publisher trigger to timer
 *	0.0.2	201103	Added heartbeat publisher
 *	0.0.1	201102	Initial poc version adapted from ROS tutorial
 *
 *	Start:
 *	roslaunch twist_mux twist_mux.launch
 *
 */



#include <twist_mux/twist_mux_class.h>

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
	ros::init(argc, argv, "twist_mux_node", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	// Override the default ros sigint handler.
	signal(SIGINT, mySigIntHandler);

	// create a TwistMux object
	TwistMux tM (&nh);

	float nap {10.0};				// Hz Default nap periode
	nh.param("/twist_mux/nap_periode", nap, nap);		// get custom nap periode
	ros::Rate naptime(nap);			// create a rate object

	// Main program loop.
	while (noShutdownRequest) {
		ros::spinOnce();			// Handle the

		naptime.sleep(); 			// Sleep for the balance of the period.
	}

	ROS_INFO("twist_mux_node: Left main program loop.");

	// Clean up before we close.
	tM.sendStop(); 					// send a x=0 z=0 cmd_vel
	ros::shutdown();				// Shutdown this node.

	return 0;
} //--[ end main ]--//

//****[ eof twist_mux_node.cpp ]****//
