### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]]
### Install turtlesim
```shell
sudo apt update
sudo apt install -y ros-humble-turtlesim
```
- Check the list of `turtlesim` executable.
```shell
ros2 pkg executables turtlesim

#turtlesim draw_square
#turtlesim mimic
#turtlesim turtle_teleop_key
#turtlesim turtlesim_node
```
### Start turtlesim
```shell
ros2 run turtlesim turtlesim_node # in one terminal
ros2 run turtlesim turtle_teleop_key # in another terminal
```
- using `list` to check the associated topics, services, and actions of nodes.
```shell
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```
### Install rqt
```shell
sudo apt update
sudo apt install ros-humble-rqt*

# use rqt
rqt
# Plugins > Services > Service Caller
```
### Try the spawn service
- Use rqt to call the `/spawn` service.
- Create another turtle in the turtlesim window.
- Give `name = turtle2`, `x = 1.0`, `y = 1.0`, `theta = 0.8`.
- Click `Call` button.
- Refresh service, you will see services related to new turtle.
![[Screenshot from 2023-12-08 11-17-34.png]]
### Try the set_pen service
- Give `turtle1` a unique pen as `r = 255`, `width = 10
- There's no way to move `turtle2`(no teleop node)
![[Screenshot from 2023-12-08 11-35-25.png]]
- In order to move `turtle2`, let's do remapping
```shell
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
![[Screenshot from 2023-12-08 11-42-07.png]]
