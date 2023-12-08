### Setup
```shell
ros2 run turtlesim turtlesim_node        # in one terminal
ros2 run turtlesim turtle_teleop_key     # in another terminal
```
### rqt_graph
```shell
rqt_graph
```
![[Screenshot from 2023-12-08 15-11-45.png]]
- The graph is depicting how the `/turtlesim` node and the `/teleop_turtle` node are communicating with each other over a topic. The `/teleop_turtle` node is publishing data (the keystrokes you enter to move the turtle around) to the `/turtle1/cmd_vel` topic, and the `/turtlesim` node is subscribed to that topic to receive the data.
### ros2 topic list
```shell
ros2 topic list
# return a list of all topics:
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
```shell
ros2 topic list -t
# return a list of all topics with the type appended in brackets:
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]

```
### ros2 topic echo
```shell
ros2 topic echo /turtle1/cmd_vel

# it returns pose data for every moment we move the turtle around
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```
![[Screenshot from 2023-12-08 16-11-02.png]]
- where `/_ros2cli_105480` is the node created by `echo` command.
- `/turtle1/cmd_vel` is publishing data over the  `cmd_vel` topic.
- `/turtlesim` and `/_ros2cli_105480` are subscribed to it.
### ros2 topic info
```shell
ros2 topic info /turtle1/cmd_vel

# it returns
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```


**Topics**:
- **Description**: Topics are named buses over which nodes exchange messages. They are used for unidirectional, streaming communication. Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
- **Example**: A sensor node might publish sensor data to a topic, while a visualization node might subscribe to the same topic to get the data and render it.