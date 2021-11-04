# <center>MG400Robot</center>

Chinese version of the README -> please [click here](./README-CN.md)

# Building
### Use git to clone the source code
```
cd $HOME/catkin_ws/src
git clone https://github.com/Dobot-Arm/MG400_ROS.git
cd $HOME/catkin_ws
```

### building
```
catkin_make
```

### activate this workspace
```
source $HOME/catkin_ws/devel/setup.bash
```

### run the rviz demo
```
roslaunch mg400_description display.launch
```
![rviz显示](./disp.jpg)

# Custom Function Development

    Msg and srv is defined in bringup. Users can control the robotic arm via those underlying commands