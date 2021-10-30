# <center>MG400Robot</center>

# 1. 源码编译
## ubuntu16.04
### 下载源码
```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/mg400.git -b kinetic-devel

cd $HOME/catkin_ws
```
### 编译
```
catkin_make
```
### 设置环境变量
```
source $HOME/catkin_ws/devel/setup.bash
```

## ubuntu18.04
###下载源码
```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/mg400.git -b melodic-devel

cd $HOME/catkin_ws
```
### 编译
```
catkin_make
```
### 设置环境变量
```
source $HOME/catkin_ws/devel/setup.bash
```

### 运行 RVIZ
```
roslaunch mg400 display.launch
```
