### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]], [[03-Using-turtlesim-package]]
### ROS2 nodes
- Each node in ROS should be responsible for a single, modular purpose. Each node can send and receive data from other nodes via topics, services, actions, or parameters.
- Using `turtlesim` as example.
```shell
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
ros2 node list

# return the node name
/teleop_turtle
/turtlesim
```
- Remapping `turtlesim` node: reassign default node properties
```shell
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
ros2 node list

# return the node name
/my_turtle  -> which is remapped from turtlesim
/teleop_turtle
/turtlesim
```
- Access more information about your nodes
```shell
ros2 node info /turtlesim

# return the node info
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```

**Node**:
- **Description**: In ROS2, a node is the fundamental building block of a ROS2 software system. It is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server.
- **Example**: A node in a robot might be responsible for controlling wheel motors, another node for handling laser range-finder data, and a third node for executing high-level navigation tasks.
