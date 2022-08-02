# installing-aarm\




install-arduino-robot-arm
install source code

git clone https://github.com/kira-Developer/arduino_robot_arm.git

Dependencies

run this instruction inside your workspace:

ake sure you installed all these packages:

for kinetic distro

$ sudo apt-get install ros-kinetic-moveit
$ sudo apt-get install ros-kinetic-joint-state-publisher ros-kinetic-joint-state-publisher-gui
$ sudo apt-get install ros-kinetic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-ros-control

for melodic distro

$ sudo apt-get install ros-melodic-moveit
$ sudo apt-get install ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
$ sudo apt-get install ros-melodic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-melodic-ros-controllers ros-melodic-ros-control

for noetic distro

$ sudo apt-get install ros-noetic-moveit
$ sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
$ sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control

Robot Arm

The robot arm has 5 joints only 4 joints can be fully controlled via ROS and Rviz, the last joint (gripper) has a default motion executed from the Arduino code directly.
Circuit diagram

circuit
Robot initial positions

positions
Configuring Arduino with ROS

    Install Arduino IDE in Ubuntu https://www.arduino.cc/en/software to install run $ sudo ./install.sh after unzipping the folder

    Launch the Arduino IDE

    Install the arduino package and ros library http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

    Make sure to change the port permission before uploading the Arduino code $ sudo chmod 777 /dev/ttyUSB0

Usage
Controlling the robot arm by joint_state_publisher

$ roslaunch robot_arm_pkg check_motors.launch

You can also connect with hardware by running:

$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

(Note: You may need to use ttyACM)
Simulation

Run the following instructions to use gazebo

$ roslaunch robot_arm_pkg check_motors.launch
$ roslaunch robot_arm_pkg check_motors_gazebo.launch
$ rosrun robot_arm_pkg joint_states_to_gazebo.py

(You may need to change the permission)

$ sudo chmod +x ~/catkin_ws/src/arduino_robot_arm/robot_arm_pkg/scripts/joint_states_to_gazebo.py
Controlling the robot arm by Moveit and kinematics

$ roslaunch moveit_pkg demo.launch

You can also connect with hardware by running:

$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

(Note: You may need to use ttyACM)
Simulation

Run the following instruction to use gazebo

$ roslaunch moveit_pkg demo_gazebo.launch
