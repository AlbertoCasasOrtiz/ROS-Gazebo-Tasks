Description
-----------

Pioneer3AT with camera and keyboard control. For each exercise change world in pioneer3at.gazebo.launch.

Exercise 1
----------

Create a world with bars in slalom and operate a pioneer3at with a camera. First from world view and then from first person view.

Execute ros+gazebo:

<code>roslaunch p2os_urdf pioneer3at.gazebo.launch</code>

Execute first person view with rqt:

<code>rosrun rqt_gui rqt_gui</code>

Operate robot with keyboard:

<code>rosrun teleop_twist_keyboard teleop_twist_keyboard.py</code>

Exercise 2
----------

Make the robot follow objects of a determined color.

Execute ros+gazebo:

<code>roslaunch p2os_urdf pioneer3at.gazebo.launch</code>

Make robot follow objects:

<code>rosrun follow_color follow_color</code>

Exercise 3
----------

Make the robot follow a goal (Red ball) and avoid obstacles (Yellow and green balls) using potential fields method.

Execute ros+gazebo:

<code>roslaunch p2os_urdf pioneer3at.gazebo.launch</code>

Make robot follow objects:

<code>rosrun potential_fields potential_fields</code>
