# <center>MG400Robot</center>

Chinese version of the README -> please [click here](./README-CN.md)

# Building
## ubuntu16.04

```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/mg400.git -b kinetic-devel

cd $HOME/catkin_ws

# building
catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```

## ubuntu18.04

### Use git to clone the source code
```
cd $HOME/catkin_ws/src
git clone https://github.com/Dobot-Arm/mg400.git -b melodic-devel
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
roslaunch mg400 display.launch
```
