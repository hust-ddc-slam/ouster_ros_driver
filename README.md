# ouster_ros_driver
Our modified ouster_ros_driver

这个代码是修改自ouster官方驱动，修改了一些lidar的配置、源代码中ring的类型等。目前，松灵车工控机、杨君宇电脑、许恩赐电脑等均采用相同的驱动版本，避免后端数据处理差异存在问题。

（带一些GPS的代码，所以不是完整的原始的sdk。后面有机会再删掉GPS的东西吧）

**系统环境**：Ubuntu20，ROS noetic。（注意，官方的代码不支持melodic版本ros，所以ubuntu直接上20了）

## 安装
1. 参考官方安装方式，安装ros相关文件：[https://github.com/ouster-lidar/ouster-ros](https://github.com/ouster-lidar/ouster-ros)
```bash
sudo apt install -y                     \
    ros-$ROS_DISTRO-pcl-ros             \
    ros-$ROS_DISTRO-rviz
```
这一步一般是安装好的，$ROS_DISTRO 换成 noetic

2. 安装其他依赖项
```bash
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake
```

编译时，这个代码还带GPS，所以遇到过serial找不到。需要再安装serial
```bash
sudo apt-get install ros-$ROS_DISTRO-serial
```

3. 编译
```bash
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## 使用
注意，在sensor.launch中我们也修改了lidar的配置。运行是这样：
```bash
roslaunch ouster_ros sensor.launch
```
注意sensor.launch启动时，需要common.launch，不要删掉了。

除了我们的修改外，还需要对lidar的ip进行设置。目前有两个ouster雷达，编号05D和新的。IP分别为：  
`sensor_hostname`: 192.168.1.5    
`sensor_hostname`: 169.254.206.157  

## 电脑端IP设置
对于169的lidar，地址、掩码、网关设置如下：  
169.254.206.xxx  
255.255.255.0  
169.254.206.1


