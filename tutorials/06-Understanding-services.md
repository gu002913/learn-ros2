like `<grpc export-func>` 
### Setup
```shell
ros2 run turtlesim turtlesim_node        # in one terminal
ros2 run turtlesim turtle_teleop_key     # in another terminal
```
### ros2 service list
```shell
ros2 service list

# it returns
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/get_type_description
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/get_type_description
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
- You will see that both nodes have the same six services with `parameters` in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of.
### ros2 service type
- To find out the type of a service, use the command `ros2 service type <service_name>`
```shell
ros2 service type /clear

# it returns
std_srvs/srv/Empty
```
- The `Empty` type means the service call sends no data when making a request and receives no data when receiving a response, just like `void`.
- Use `list` command to see the types of all the active services.
```shell
ros2 service list -t

# it returns
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/teleop_turtle/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/teleop_turtle/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/teleop_turtle/get_parameters [rcl_interfaces/srv/GetParameters]
/teleop_turtle/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/teleop_turtle/list_parameters [rcl_interfaces/srv/ListParameters]
/teleop_turtle/set_parameters [rcl_interfaces/srv/SetParameters]
/teleop_turtle/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```
### ros2 service find
- To find all services of a specific type, use this command`ros2 service find <type_name>`
```shell
ros2 service find std_srvs/srv/Empty

# it returns
/clear
/reset
```
### ros2 interface show
- To know the request and response arguments of service, use command `ros2 interface show <type_name>`
```shell
ros2 interface show turtlesim/srv/Spawn

# it returns
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name

```
- arguments above `---` is the request variables, arguments below `---` is the response variables (return).
### ros2 service call
- To call the service, use command `ros2 service call <service_name> <service_type> <arguments>`
- The `<arguments>` part is optional. For example, `Empty` typed service has no arguments.
```shell
ros2 service call /clear std_srvs/srv/Empty
```
![[Screenshot from 2023-12-11 15-56-59 1.png]]
![[Screenshot from 2023-12-11 16-01-59.png]]
- This command clears the turtlesim window of any lines your turtle has drawn. At the meantime, the terminal prints the response arguments we expected.
- More practice by calling `/spawn` and setting request arguments.
```shell
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```
![[Screenshot from 2023-12-11 16-06-03.png]]
- The turtlesim window will update with the newly spawned turtle right away:

**Service**:
- **Description**: Services provide a way for nodes to perform remote procedure calls (RPC). A service client sends a request message to a service server, which processes the request and sends back a response.
- **Example**: A node providing a map service could send a map to another node that requests it. This is similar to a function call in programming languages.