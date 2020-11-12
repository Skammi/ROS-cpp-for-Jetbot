# ROS-for-Jetbot-cpp

ROS nodes and driver libraries for NVIDIA JetBot with Jetson Nano. The libraries are aimed for the Jetbot Kit provided by WaveShare.

# Synopsis
This is work still under construction, although you can control the (WaveShare) Jetbot with a game controller.
The code is in 2 folders one for the Driver Libraries and one for the ROS nodes.

## Driver Libraries
The driver library structure is based on a Library set I first made for the RPI.
Here a short overview:
- i2c --> Library provides the communication to the i2c bus
- ina219 --> Library provides the communication to the ina219 chip uses i2c.
- ina219_poc --> Program to test the ina219 library.
- pwm9685 --> Library provides the communication to the pwm9685 chip uses i2c.
- motorcap --> Library to run the motors, uses pwm9685 and i2c.
- motorcap_poc -> Program to test the motorcap library.

**Note**: The libraries need to be compiled with the Position Independent code option (-fPIC)

## Ros-nodes
The Ros nodes are split over a Linux PC and the Jetson Nano. Each package where appropriate has launch- and parameter file. There is also a package with to start-up launch files on for the PC and one for the Jetson.
Here a short overview:
- jetbot_msg -> Contains the messages used. Not all are necessary used now.
- teleop_joy_twist -> Uses the info published by the standard joy package and convert it to a telep/cmd_vel message.
- twist_mux -> Takes a teleop or auto cmd_vel publish that it also published a heartbeat.
- jetbot-node -> Takes the cmd_vel and steers the motors. It also subscribes to the heartbeat and stops the motors when no heartbeat is received for a specific periode.
- jetbot_stats -> Publishes statistics from the jetbot.
- jetbot-start -> Contains launch files one to start the nodes on the Jetbot and one to start the nodes on the PC.

### Graphical Overview

![Alt text](jetbotDataFlowDiagram.tiff?raw=true "Jetbot data flow diagram")

