### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]], [[03-Using-turtlesim-package]]
### Running a Launch File
```shell
ros2 launch turtlesim multisim.launch.py
```
- This command will run the following launch file:
```python
# turtlesim/launch/multisim.launch.py
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "turtlesim1", 
            package='turtlesim', 
            executable='turtlesim_node', 
            output='screen'),
        launch_ros.actions.Node(
            namespace= "turtlesim2", 
            package='turtlesim',   
            executable='turtlesim_node', 
            output='screen'),
    ])
```
- This will run two turtlesim nodes.
### Control the multi-turtlesim nodes
- In the new terminal:
```shell
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
- In the new terminal:
```shell
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
![[Screenshot from 2023-12-28 16-10-46.png]]
### Recall what we learnt
- If I wanna reset one of the turtlesim nodes, service can be called.
```shell
ros2 service list -t
```
- which returns:
```shell
/turtlesim1/clear [std_srvs/srv/Empty]
/turtlesim1/kill [turtlesim/srv/Kill]
/turtlesim1/reset [std_srvs/srv/Empty]
/turtlesim1/spawn [turtlesim/srv/Spawn]
/turtlesim1/turtle1/set_pen [turtlesim/srv/SetPen]
/turtlesim1/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtlesim1/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim1/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim1/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim1/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim1/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim1/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim1/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/turtlesim2/clear [std_srvs/srv/Empty]
/turtlesim2/kill [turtlesim/srv/Kill]
/turtlesim2/reset [std_srvs/srv/Empty]
/turtlesim2/spawn [turtlesim/srv/Spawn]
/turtlesim2/turtle1/set_pen [turtlesim/srv/SetPen]
/turtlesim2/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtlesim2/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim2/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim2/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim2/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim2/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim2/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim2/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```
- Lets say `reset`the second turtle.
```shell
ros2 service call turtlesim2/reset std_srvs/srv/Empty
```
- which returns:
```shell
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()
```
- It comes out the reset of the second turtle while no influence on the first one.