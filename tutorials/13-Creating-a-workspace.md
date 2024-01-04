### Prerequisites
- ROS2 installation
- colcon installation
- git installation
- turtlesim installation
### Install rosdep
```shell
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```
### Move to the directory
```shell
cd ~/path/to/workspace/src
```
### 



### Dependencies
- Run `rosdep install` to install dependencies. Typically, this is run over a workspace with many packages in a single call to install all dependencies. A call for that would appear as the following, if in the root of the workspace with directory `src` containing source code.
```shell
rosdep install --from-paths src -y --ignore-src
```
