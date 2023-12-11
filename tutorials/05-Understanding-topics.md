like `<channel>` in `redis`
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
- which means that in the package `geometry_msgs` there is a `msg` called `Twist`.
```shell
ros2 interface show geometry_msgs/msg/Twist

# it returns

# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z

```
- This tells you that the `turtlesim` node is expecting a message `data` with two vectors, `Vector3 linear` and `Vector3 angular`, of three elements each. 
- Recall the `topic echo` we used. It's the same data structure.
### ros2 topic pub
- like `redis-cli` > `publish <channel> '<data>'` we can do same thing in ROS: `ros2 topic pub <topic_name> <msg_type> '<args>'`.
```shell
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
- `--once` is an optional argument meaning "publish one message then exit".
![[Screenshot from 2023-12-11 13-55-43.png]]
- The turtle move with `vel` given(both linear and angular) in a short period as we expected. The duration for which the turtle will move with the specified velocity depends on the simulation environment and the physics settings.
- In order to get the turtle to keep moving at a given frequency, we can run:
```shell
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
- This command publish the velocity with a steady stream at 1 Hz.
![[Screenshot from 2023-12-11 14-03-36.png]]
```shell
ros2 topic echo /turtle1/pose
```
- Recheck `rqt_graph`.
![[Screenshot from 2023-12-11 14-17-35.png]]
- You can see that the `/turtlesim` node is also publishing to the `pose` topic, which the new `echo` node has subscribed to.
### ros2 topic hz
- View the rate at which data is published using:
```shell
ros2 topic hz /turtle1/pose # in one terminal
ros2 topic hz /turtle1/cmd_vel # in the other terminal
```

![[Screenshot from 2023-12-11 14-24-15.png]]


**Topics**:
- **Description**: Topics are named buses over which nodes exchange messages. They are used for unidirectional, streaming communication. Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
- **Example**: A sensor node might publish sensor data to a topic, while a visualization node might subscribe to the same topic to get the data and render it.