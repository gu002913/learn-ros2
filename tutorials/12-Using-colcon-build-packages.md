### Background
- `colcon` is an iteration on the ROS build tools `catkin_make`, `catkin_make_isolated`, `catkin_tools` and `ament_tools`.
### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]]
- Install `colcon`:
```shell
sudo apt install python3-colcon-common-extenstions
```
### Create a workspace
```shell
mkdir -p ~/path/to/your/workspace/ros2_ws/src
cd ~/path/to/your/workspace/ros2_ws/
```
- Let's clone the examples repository into the `src` directory.
```shell
git clone https://github.com/ros2/examples src/examples -b humble
```
- Now the source code is like:
```shell
.
└── src
    └── examples
        ├── CONTRIBUTING.md
        ├── LICENSE
        ├── rclcpp
        ├── rclpy
        └── README.md

4 directories, 3 files
```
### Source the underlay
- An "overlay" is a separate workspace (e.g., "ros2_ws") that you create on top of your existing ROS 2 installation (the underlay).
- The overlay is where you develop and work on your custom ROS 2 packages and projects. You organize your code, configurations, and custom packages within this workspace.
- When you work within your overlay (ros2_ws), you can make changes and additions to your ROS 2 packages without affecting the underlying ROS 2 installation (the underlay).
- This separation is useful because it allows you to manage and iterate on your custom packages independently without worrying about breaking the core ROS 2 installation or other packages.
### Build the workspace
```shell
colcon build --symlink-install
```
- `colcon` supports the option `--symlink-install`. This allows the installed files to be changed by changing the files in the `source` space (e.g. Python files or other non-compiled resources) for faster iteration.
- After the build is finished, we should see the `build`, `install`, and `log` directories:
```shell
.
├── build
├── install
├── log
└── src

4 directories, 0 files
```
### Run tests
```shell
colcon test
```
### Source the environment
- When colcon has completed building successfully, the output will be in the `install` directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the `install` directory to help set up the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.
```shell
souce install/setup.bash
```
### Try a demo
- Let's run a sub-pub demo:
```shell
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```
- In another terminal, run:
```shell
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```
- which returns:
```shell
[INFO] [1703831815.435871410] [minimal_publisher]: Publishing: 'Hello, world! 0'
[INFO] [1703831815.935838372] [minimal_publisher]: Publishing: 'Hello, world! 1'
[INFO] [1703831816.435879983] [minimal_publisher]: Publishing: 'Hello, world! 2'
[INFO] [1703831816.935851822] [minimal_publisher]: Publishing: 'Hello, world! 3'
[INFO] [1703831817.435888248] [minimal_publisher]: Publishing: 'Hello, world! 4'
[INFO] [1703831817.935852962] [minimal_publisher]: Publishing: 'Hello, world! 5'
[INFO] [1703831818.435897645] [minimal_publisher]: Publishing: 'Hello, world! 6'
[INFO] [1703831818.935919762] [minimal_publisher]: Publishing: 'Hello, world! 7'
[INFO] [1703831819.435903539] [minimal_publisher]: Publishing: 'Hello, world! 8'
[INFO] [1703831819.935871611] [minimal_publisher]: Publishing: 'Hello, world! 9'
[INFO] [1703831820.435866082] [minimal_publisher]: Publishing: 'Hello, world! 10'
...
```
- the listener shows:
```shell
[INFO] [1703831815.436366548] [minimal_subscriber]: I heard: 'Hello, world! 0'
[INFO] [1703831815.936011627] [minimal_subscriber]: I heard: 'Hello, world! 1'
[INFO] [1703831816.436226880] [minimal_subscriber]: I heard: 'Hello, world! 2'
[INFO] [1703831816.936022101] [minimal_subscriber]: I heard: 'Hello, world! 3'
[INFO] [1703831817.436226794] [minimal_subscriber]: I heard: 'Hello, world! 4'
[INFO] [1703831817.936015394] [minimal_subscriber]: I heard: 'Hello, world! 5'
[INFO] [1703831818.436271547] [minimal_subscriber]: I heard: 'Hello, world! 6'
[INFO] [1703831818.936278597] [minimal_subscriber]: I heard: 'Hello, world! 7'
[INFO] [1703831819.436259320] [minimal_subscriber]: I heard: 'Hello, world! 8'
[INFO] [1703831819.936046085] [minimal_subscriber]: I heard: 'Hello, world! 9'
[INFO] [1703831820.436026876] [minimal_subscriber]: I heard: 'Hello, world! 10'
...
```
