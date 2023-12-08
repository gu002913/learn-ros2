### Add sourcing to shell startup script
```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
so that you don’t  have to source the setup file every time you open a new shell.
### Check environment variables
- `ROS_DISTRO` and `ROS_VERSION`
```shell
printenv | grep -i ROS
```
- The `ROS_DOMAIN_ID`, like 11 or something you prefer.
```shell
export ROS_DOMAIN_ID=<your_domain_id>
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```
- The `ROS_LOCALHOST_ONLY` .
```shell
export ROS_LOCALHOST_ONLY=1
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```