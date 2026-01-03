# ROS2 + Gazebo Project 


**Instructions to install ROS2 deb packages on Ubuntu**  
LINK: [https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

**Install the build tool for ROS2**
```
sudo apt install python3-colcon-common-extensions
```

**Install Gazebo for ROS2 Kilted**
```
sudo apt-get install ros-kilted-ros-gz
```

**Run this command on every new shell to have access to ROS 2 commands**
``` 
source /opt/ros/kilted/setup.bash
```
 
**Or add this to the shell startup script** 
```
echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
```

**Run this command inside the 'ros_ws' directory to compile the workspace packages**
```
colcon build
```

**Run this command inside the 'ros_ws' directory to make ROS aware of the newly compiled packages**
```
source /install/setup.bash
```

**Run this command inside the 'ros_ws' directory to view the simulation**
```
ros2 launch diff_drive bot_launch.py
```
