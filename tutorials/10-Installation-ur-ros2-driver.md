### Prerequisites
- Before starting these tutorials, install ROS2 by following the instructions on [[01-Installation-ubuntu]], [[02-Configuring-environment]]
### Build from source
- Make sure that `colcon`, its extensions and `vcs` are installed.
```shell
sudo apt install python3-colcon-common-extensions python3-vcstool
```
- Create a new ROS2 workspace, `path/to/workspace` depends on your own.
```shell
export COLCON_WS=~/path/to/workspace/ros2-ur-driver
mkdir -p $COLCON_WS/src
```
- Clone relevant packages, install dependencies, compile, and source workspace by using:
```shell
cd $COLCON_WS
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- Bug encountered: 
- [ ] Failed for `python3-rosidl-generator-py`
```shell
conda deactivate # possible solution
```

```shell
gedit ~/.bashrc
# comment anaconda stuff like

# export PATH=/home/dingyi/anaconda3/bin:$PATH

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
#__conda_setup="$('/home/dingyi/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
#if [ $? -eq 0 ]; then
#    eval "$__conda_setup"
#else
#    if [ -f "/home/dingyi/anaconda3/etc/profile.d/conda.sh" ]; then
#        . "/home/dingyi/anaconda3/etc/profile.d/conda.sh"
#    else
#        export PATH="/home/dingyi/anaconda3/bin:$PATH"
#    fi
#fi
#unset __conda_setup
# <<< conda initialize <<<
```
- [ ] `joint_state_publisher_gui` not found
```shell
sudo apt install ros-humble-joint-state-publisher-gui
source /opt/ros/humble/setup.bash
```
- [ ] Other bug related to ros2-distro
```shell
# check git branch
cd $COLCON_WS/src/Universal_Robots_ROS2_Description/
git branch
git switch humble # if current branch is not humble

cd $COLCON_WS/src/Universal_Robots_ROS2_Driver/
git branch
git switch humble # if current branch is not humble
```
