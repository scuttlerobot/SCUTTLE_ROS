Notes from Christian: Not final and just for basic guidance

This is a package for controlling the motors on scuttle and also reading basic encoder data

This is very messy and just an experiment.

Prereqs: This was made for ros Kinetic but seems compatible with Melodic
Installed: ROS Melodic/Kinetic on some flavor of ubuntu to match (Server 18.04 or Ubuqitity Robotics image Kinetic-Only)
Scuttle: Wired via the Wiring Guide (May be wrong so be careful!)
	This includes encoders at addresses x40 and x41 respectively

Packages needed from ros for teleop driving
teleop_twist_joystick or teleop_twist_keyboard - Path differs here, Ill give keyboard instructions as that is easier