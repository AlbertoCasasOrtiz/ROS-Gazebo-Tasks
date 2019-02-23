Description
-----------

Pioneer3AT with camera and keyboard control.

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
