# Smach (State Machine) demonstration
This workspace demonstrates a basic usage of Smach for a mobile base.
It has two functionalities:
1. Move to goal ( in a straight line), when reached or crossed the goal
2. Move in circle.

## Setup
_Create a workspace:_ <br>
`mkdir basic_ws/src` <br>
_Place the file in the src:_ <br>
`cd basic_ws/src` <br>
_Make:_<br>
`cd ..`<br>
`catkin_make`<br>
`source devel/setup.bash`<br>
_Run the commands:_<br>
Launch robot model in gazebo:<br>
`roslaunch mobile_manipulator_body base_gazebo_control.launch`<br>
Run the smach file:<br>
`rosrun mobile_manipulator_body smach_basic.py`<br>

## Dependencies:
ROS Noetic<br>
Smach<br>

## Result:<br>

![Output for transitions](results/transition.png)<br>
![Smach viewer Tree](results/tree.png)
