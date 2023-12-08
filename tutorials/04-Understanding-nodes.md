### The ROS2 graph
![[Screenshot from 2023-12-08 13-40-13.png]]
### ROS2 nodes
- Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.
- Using `turtlesim` as example.
```shell
ros2 run turtlesim turtlesim_node
ros2 node list

# return the node name
# /turtlesim
```
