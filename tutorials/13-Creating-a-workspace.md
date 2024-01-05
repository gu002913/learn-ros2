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
### Clone a sample repo
```shell
git clone https://github.com/ros/ros_tutorials.git -b humble
```
### Resolve dependencies
- Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.
- From the root of your workspace (`ros2_ws`), run the following command:
```shell
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```
- If you already have all your dependencies, the console will return:
```shell
#All required rosdeps installed successfully
```
### Build the workspace with colcon
- From the root of your workspace (`ros2_ws`), you can now build your packages using the command:
```shell
colcon build
```
- which returns:
```shell
[1.481s] WARNING:colcon.colcon_core.package_selection:Some selected packages are already built in one or more underlay workspaces:
	'turtlesim' is in: /opt/ros/humble
If a package in a merged underlay workspace is overridden and it installs headers, then all packages in the overlay must sort their include directories by workspace order. Failure to do so may result in build failures or undefined behavior at run time.
If the overridden package is used by another package in any underlay, then the overriding package in the overlay must be API and ABI compatible or undefined behavior at run time may occur.

If you understand the risks and want to override a package anyways, add the following to the command line:
	--allow-overriding turtlesim

This may be promoted to an error in a future release of colcon-override-check.
Starting >>> turtlesim
Finished <<< turtlesim [20.1s]                       

Summary: 1 package finished [21.1s]
```
- since we already have `turtlsime` compiled in our ROS2 source or underlay, the warning comes out to check if we wanna override. Just ignore it and keep going with the overlay.
```shell
ls

#it returns
build  install  log  src
```
- The `install` directory is where your workspace’s setup files are, which you can use to source your overlay.
### Source the overlay
```shell
source install/local_setup.bash
```
### Modify the overlay
- You can modify `turtlesim` in your overlay by editing the title bar on the turtlesim window. To do this, locate the `turtle_frame.cpp` file in `~/ros2_ws/src/ros_tutorials/turtlesim/src`. Open `turtle_frame.cpp` with your preferred text editor.
- On line 52 you will see the function `setWindowTitle("TurtleSim");`. Change the value `"TurtleSim"` to `"Dingyi-TurtleSim"`, and save the file.
- Return to the first terminal where you ran `colcon build` earlier and run it again.
- Return to the second terminal (where the overlay is sourced) and run turtlesim again
- To see that your underlay is still intact, open a brand new terminal and source only your ROS 2 installation. Run turtlesim again:
![[Screenshot from 2024-01-05 14-00-04.png]]
- You can see that modifications in the overlay did not actually affect anything in the underlay.
