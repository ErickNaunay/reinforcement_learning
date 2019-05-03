# reinforcement_learning
The university of OKlahoma.
Sping - 2019
Project developed for CS4023, Introduction to Intelligent Robotics

Running the code.-
Both the TurtleBot computer and the CSN Linux box must be connected to the Wi-Fi network “CS4023”. 

Find out the IP address of the TurtleBot computer and the CSN Linux box by running the command the following command in the terminal of each computer.

ifconfig

Log into the Turtlebot computer from the CSN Linux box by using ssh:

    ssh student@<TURTLEBOT_IP>

In the the laptop attached to the TurtleBot, the robot navigation launch packages must to be located inside the catkin_ws/ src directory, then command catkin_make must be typed for building the packages inside catkin_ws/.

Export the variables inside your workspace setup script:

echo export ROS_MASTER_URI=http://IP_OF_TURTLEBOT:11311 >> ~/.bashrc

echo export ROS_HOSTNAME=IP_OF_PC >> ~/.bashrc

Remote computer:

Make sure the TurtleBot is turned on. Through ssh, startup the TurtleBot by running:

roslaunch turtlebot_bringup minimal.launch –screen

Set the 3D sensors of the TurtleBot to “kinect”:

export TURTLEBOT_3D_SENSOR=kinect

Startup the 3D camera:
roslaunch turtlebot_bringup 3dsensor.launch

SETUP Catkin package (in a new shell)
1) Create or use a catkin workspace called "catkin_ws"

clone the repository into the ~/catkin_ws/src/ folder

cd ~/catkin_ws
catkin_make

Each shell opened has to run the following command before launching any ros node: 
. ~/catkin_ws/devel/setup.bash

For running the robot real simulation:
export DISPLAY=:0
roslaunch qlearning_project qlearning.launch --screen

