p2os: [![Build Status](https://travis-ci.org/allenh1/p2os.svg?branch=master)](https://travis-ci.org/allenh1/p2os)
====

P2OS driver for the Pioneer robots. This driver was originally written for Player/Stage by Brian Gerkey.

p2os_driver
-----------

Essential to P2OS is the driver. This controls the interface for the P2OS controller. 

Jade Build:

[![Build Status](http://build.ros.org/job/Jbin_uT32__p2os_driver__ubuntu_trusty_i386__binary/1/badge/icon)](http://build.ros.org/job/Jbin_uT32__p2os_driver__ubuntu_trusty_i386__binary/1/)

Kinetic Build:

[![Build Status](http://build.ros.org/job/Ksrc_uX__p2os_driver__ubuntu_xenial__source/badge/icon)](http://build.ros.org/job/Ksrc_uX__p2os_driver__ubuntu_xenial__source/)

p2os_launch
-----------

Relevant ROS launch files for the Robot. 

Jade Build:

[![Build Status](http://build.ros.org/job/Jbin_uT32__p2os_launch__ubuntu_trusty_i386__binary/1/badge/icon)](http://build.ros.org/job/Jbin_uT32__p2os_launch__ubuntu_trusty_i386__binary/1/)

Kinetic Build:

[![Build Status](http://build.ros.org/job/Ksrc_uX__p2os_launch__ubuntu_xenial__source/badge/icon)](http://build.ros.org/job/Ksrc_uX__p2os_launch__ubuntu_xenial__source/)

p2os_teleop
-----------

Control the robot with a joystick or keyboard. 

Jade Build:

[![Build Status](http://build.ros.org/job/Jbin_uT32__p2os_teleop__ubuntu_trusty_i386__binary/1//badge/icon)](http://build.ros.org/job/Jbin_uT32__p2os_teleop__ubuntu_trusty_i386__binary/1/)

Kinetic Build:

[![Build Status](http://build.ros.org/job/Ksrc_uX__p2os_teleop__ubuntu_xenial__source/badge/icon)](http://build.ros.org/job/Ksrc_uX__p2os_teleop__ubuntu_xenial__source/)

p2os_urdf
---------

Allows you to see the robot within RVIZ for navigation purposes. 

Jade Build:

[![Build Status](http://build.ros.org/job/Jsrc_uT__p2os_urdf__ubuntu_trusty__source/2/badge/icon)](http://build.ros.org/job/Jsrc_uT__p2os_urdf__ubuntu_trusty__source/2/)

Kinetic Build:

[![Build Status](http://build.ros.org/job/Ksrc_uX__p2os_urdf__ubuntu_xenial__source/badge/icon)](http://build.ros.org/job/Ksrc_uX__p2os_urdf__ubuntu_xenial__source)

Exercise 1
----------

Create a world with bars in slalom and operate a pioneer3at with a camera. First from world view and then from first person view.

Execute ros+gazebo: roslaunch p2os_urdf pioneer3at.gazebo.launch
Execute first person view with rqt: rosrun rqt_gui rqt_gui 
Operate robot with keyboard: rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Exercise 2
----------

Make the robot follow objects of a determined color.

Execute ros+gazebo: roslaunch p2os_urdf pioneer3at.gazebo.launch
Make robot follow objects: rosrun follow_color follow_color 
