### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]], [[03-Using-turtlesim-package]]
### Setup
```shell
ros2 run turtlesim turtlesim_node        # in one terminal
ros2 run turtlesim turtle_teleop_key     # in another terminal
```
### ros2 action list
- where `/turtle1/rotate_absolute` is the `<action_name>` and `turtlesim/action/RotateAbsolute` is the `<action_type>`.
```shell
ros2 action list -t

# it returns
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```
### ros2 action info
```shell
ros2 action info /turtle1/rotate_absolute

# it returns
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```
### ros2 interface show
```shell
ros2 interface show turtlesim/action/RotateAboslute

# it returns
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```
### ros2 action send_goal
- send an action goal using `ros2 action send_goal <action_name> <action_type> <values>` command.
```shell
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```
- which returns:
```shell
Waiting for an action server to become available...
Sending goal:
     theta: 1.57

Goal accepted with ID: b44a56f5308f4655833f9795ccd8aafa

Result:
    delta: -1.5679999589920044

Goal finished with status: SUCCEEDED
```
- All goals have a unique ID, shown in the return message. You can also see the result, a field with the name `delta`, which is the displacement to the starting position.
- To see the feedback of this goal, add `--feedback` to the `ros2 action send_goal` command:
```shell
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```
- which returns:
```shell
Waiting for an action server to become available...
Sending goal:
     theta: -1.57

Feedback:
    remaining: -3.133185386657715

Goal accepted with ID: d67714e62ebc421caff9f145a3d6d4c8

Feedback:
    remaining: -3.117185354232788

Feedback:
    remaining: -3.1011853218078613

Feedback:
    remaining: -3.0851855278015137

Feedback:
    remaining: -3.069185256958008

Feedback:
    remaining: -3.05318546295166

...

Feedback:
    remaining: -0.061185359954833984

Feedback:
    remaining: -0.04518532752990723

Feedback:
    remaining: -0.02918529510498047

Feedback:
    remaining: -0.013185381889343262

Result:
    delta: 3.119999885559082

Goal finished with status: SUCCEEDED
```

**Action**: A cleaning task is implemented as an action. The main control node sends a goal to start cleaning a specific room, and the action server (part of the navigation node) manages this task, providing feedback on progress and eventually the result when the task is complete. (like the redis channel pub-sub for a specific task)
