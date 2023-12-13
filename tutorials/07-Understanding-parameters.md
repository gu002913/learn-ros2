### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]], [[03-Using-turtlesim-package]]
### Setup
```shell
ros2 run turtlesim turtlesim_node        # in one terminal
ros2 run turtlesim turtle_teleop_key     # in another terminal
```
### ros2 param list
```shell
ros2 param list

# it returns
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  start_type_description_service
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  holonomic
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time
```
- Every node has the parameter `use_sim_time`; it’s not unique to turtlesim. Based on their names, it looks like `/turtlesim`’s parameters determine the background color of the turtlesim window using RGB color values.
### ros2 param get
- To display the type and current value of a parameter, use the command: `ros2 param get <node_name> <parameter_name>`.
- Let’s find out the current value of `/turtlesim`’s parameter `background_g`:
```shell
ros2 param get /turtlesim background_g

# it returns
Integer value is: 86
```
### ros2 param set
- To change a parameter’s value at runtime, use the command: `ros2 param set <node_name> <parameter_name> <value>`.
```shell
ros2 param set /turtlesim background_r 150

# it returns 
Set parameter successful
```
![[Screenshot from 2023-12-11 16-51-33.png]]
### ros2 param dump
- To view all of a node’s current parameter values by using the command: `ros2 param dump /turtlesim > turtlesim.yaml` and save it to your current directory.
```yaml
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 69
    holonomic: false
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    start_type_description_service: true
    use_sim_time: false
```
### ros2 param load
- You can load parameters from a file to a currently running node using the command: `ros2 param load /turtlesim turtlesim.yaml`
```shell
ros2 param load /turtlesim turtlesim.yaml

# it returns
Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
Set parameter start_type_description_service failed: parameter 'start_type_description_service' cannot be set because it is read-only
Set parameter use_sim_time successful
```
- Read-only parameters can only be modified at startup and not afterwards, that is why there are some warnings for the “qos_overrides” parameters.
### Load parameter file on node startup
- To start the same node using your saved parameter values, use: `ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>`
```shell
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```
- The turtlesim window should appear as usual, but with the purple background you set earlier.


**Parameter**:
- **Description**: Parameters are dynamic values that can be changed at runtime. They are often used to configure the settings of a node, such as setting thresholds, tuning control loops, or setting algorithm parameters.
- **Example**: A camera node might have parameters for adjusting the exposure time, gain, and resolution of the camera.
- **Difference compared to service request arguments**:  parameters are more like member variables to a class, or static variables, which is suited for configuring the overall behavior of a node and persisting across multiple invocations.