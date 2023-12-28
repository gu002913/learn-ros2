### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]], [[03-Using-turtlesim-package]]
### Setup
```shell
ros2 `run` turtlesim turtlesim_node        # in one terminal
ros2 run turtlesim turtle_teleop_key     # in another terminal
```
- Start `rqt_console` in a new terminal
```shell
ros2 run rqt_console rqt_console
```
- The first section of the console is where log messages from your system will display.
- In the middle you have the option to filter messages by excluding severity levels. You can also add more exclusion filters using the plus-sign button to the right.
- The bottom section is for highlighting messages that include a string you input. You can add more filters to this section as well.
### Messages on rqt_console
- Let’s have the turtle run into the wall.
```shell
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"
```
![[Screenshot from 2023-12-28 15-21-25.png]]
### Logger levels
```
Fatal
Error
Warn
Info
Debug
```
- `Fatal` messages indicate the system is going to terminate to try to protect itself from detriment.
    
- `Error` messages indicate significant issues that won’t necessarily damage the system, but are preventing it from functioning properly.
    
- `Warn` messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don’t harm functionality outright.
    
- `Info` messages indicate event and status updates that serve as a visual verification that the system is running as expected.
    
- `Debug` messages detail the entire step-by-step process of the system execution.