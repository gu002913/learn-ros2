### System setup
- Set locale with supports `UTF-8`.
```shell
# set locale
locale
sudo apt update && sudo apt install locales
sudo locale-gen enUS enUS.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UFT-8
export LANG=en_US.UTF-8

locale # verify settings
```
- Enable required repositories.
```shell
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```
- Add the ROS2 GPG key with apt.
```shell
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
- Add the repository to your sources list.
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
- Install development tools.
```shell
sudo apt update && sudo apt install -y ros-dev-tools
```

### Install ROS2
- Update apt repository.
```shell
sudo apt update && sudo apt upgrade
sudo apt install -y ros-humble-desktop
```
- Try talker-listener examples - talker.
```shell
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
- In another terminal - listener.
```shell
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

### Uninstall 
```shell
# remove ros2
sudo apt remove ~nros-humble-* && sudo apt autoremove
# remove the repository
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
```
