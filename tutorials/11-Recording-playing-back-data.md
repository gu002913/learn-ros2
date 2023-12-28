### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]], [[03-Using-turtlesim-package]]
- You should have `ros2 bag` installed as a part of the regular ROS2 setup.
### Setup
```shell
ros2 `run` turtlesim turtlesim_node        # in one terminal
ros2 run turtlesim turtle_teleop_key     # in another terminal
```
- `cd` to your ROS2 workspace.
```shell
mkdir bag_files
cd bag_files
```
- To check topic:
```shell
ros2 topic list

# which returns
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
- To see the data that `/turtle1/cmd_vel` is publishing:
```shell
ros2 topic echo /turtle1/cmd_vel
```
### ros2 bag record
- To record the data published to a topic use the command `ros2 bag record <topic_name>`. Don't forget to move into the `bag_files` directory created earlier.
```shell
ros2 bag record /turtle1/cmd_vel
```
- which returns:
```shell
[INFO] [1703753593.216251156] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1703753593.218243008] [rosbag2_storage]: Opened database 'rosbag2_2023_12_28-16_53_13/rosbag2_2023_12_28-16_53_13_0.db3' for READ_WRITE.
[INFO] [1703753593.220338087] [rosbag2_recorder]: Listening for topics...
[INFO] [1703753593.220350179] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1703753593.221528623] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [1703753593.221601200] [rosbag2_recorder]: Recording...
[INFO] [1703753593.221799247] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```
- Now we can use `teleop_key` to manipulate the turtle whatever you want. And `ctrl+c` to stop recording when you finished manipulation.
```shell
[INFO] [1703753633.176063437] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1703753633.178022878] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1703753633.178189448] [rosbag2_recorder]: Recording stopped
[INFO] [1703753633.180334889] [rosbag2_recorder]: Recording stopped
```
- The data will be accumulated in a new bag directory with a name in the pattern of `rosbag2_2023_12_28-16_53_13`. This directory will contain a `metadata.yaml` along with the bag file in the recorded format.
- Check the bag file info:
```shell
ros2 bag info rosbag2_2023_12_28-16_53_13/

# it returns:
Files:             rosbag2_2023_12_28-16_53_13_0.db3
Bag size:          41.0 KiB
Storage id:        sqlite3
Duration:          19.56s
Start:             Dec 28 2023 16:53:29.946 (1703753609.946)
End:               Dec 28 2023 16:53:49.3 (1703753629.3)
Messages:          220
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 220 | Serialization Format: cdr
```
### Record multiple topics
```shell
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose
```
- The `-o` option allows you to choose a unique name for your bag file. The following string, in this case `subset`, is the file name.
- Same as the previous section, which returns:
```shell
[INFO] [1703755324.446845107] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1703755324.448037595] [rosbag2_storage]: Opened database 'subset/subset_0.db3' for READ_WRITE.
[INFO] [1703755324.448566732] [rosbag2_recorder]: Listening for topics...
[INFO] [1703755324.448577403] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1703755324.449881768] [rosbag2_recorder]: Subscribed to topic '/turtle1/pose'
[INFO] [1703755324.453234931] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [1703755324.453302243] [rosbag2_recorder]: Recording...
[INFO] [1703755324.453496355] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```
- To check the bag info:
```shell
ros2 bag info subset/

# it returns:
Files:             subset_0.db3
Bag size:          257.4 KiB
Storage id:        sqlite3
Duration:          49.952s
Start:             Dec 28 2023 17:22:04.465 (1703755324.465)
End:               Dec 28 2023 17:22:54.418 (1703755374.418)
Messages:          3648
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 525 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3123 | Serialization Format: cdr
```
### ros2 bag play
```shell
ros2 bag play subset
```
- Your turtle will follow the same path you entered while recording (though not 100% exactly; turtlesim is sensitive to small changes in the system’s timing). 
- Try `ros2 service call /reset std_srvs/srv/Empty`before play back data if weird trajectory generated. 