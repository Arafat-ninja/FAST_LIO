# FAST_LIO
仅用于学习



# **fast-lio安装编译，传感器使用方法，真机测试效果**

## 预安装库以及传感器驱动

**ubuntu版本要求：ubuntu16.04以及之后的版本**

**ros版本要求：ros melodic以及之后的版本**

**对于ubuntu18.04操作系统来说默认安装的pcl库和eigen库已经足够支持FAST-LIO的正常运行**

### **pcl库的安装，以及验证安装是否成功的方法**

**pcl安装链接（官方库）：https://pointclouds.org/downloads/**

**linux版本的安装命令：**

```shell
$ sudo apt install libpcl-dev
```

**判断pcl库是否安装成功：创建一个目录：写一个cpp文件：test_pcl.cpp 再写一个CMakeLists.txt** 

**代码内容已上传百度网盘：链接: https://pan.baidu.com/s/1XTZKh9giUHX28USZ949STw  密码: g5ci**

**下载后在test_pcl目录中新建build目录。**

**在build目录下先进行 :**

```shell
$ cmake ..
```

**在进行 ：**

```shell
$ make
```

**最后执行test_pcl 可执行文件即可**

### **Eigen库安装方法以及版本确认方法**

**运行fast-lio要求eigen库的版本要：Eigen >= 3.3.4**

**安装命令：**

```shell
$ sudo apt install libeigen3-dev
```

**查看Eigen库版本方法①：**

```shell
$ cat /usr/include/eigen3/Eigen/src/Core/util/Macros.h
```

**查看Eigen库版本方法②：**

```shell
$ pkg-config --modversion eigen3
```



### **Livox ROS Driver(览沃ROS驱动程序中文说明)**

**在运行Livox ros driver之前需要安装ros 和 livox SDK**

**livox SDK中文安装说明：https://github.com/Livox-SDK/Livox-SDK/blob/master/README_CN.md**

**安装步骤：Livox SDK 依赖于 cmake 。你可以通过 apt 工具安装这些依赖包 :**

```shell
$ sudo apt install cmake
$ git clone https://github.com/Livox-SDK/Livox-SDK.git
$ cd Livox-SDK
$ cd build && cmake ..
$ make
$ sudo make install
```



**以上步骤就是完成了livox SDK的安装**

**现在安装livox ros driver**

**首先从github下载驱动包：**

**进入livox ros 驱动包的工作空间后进行编译**

```shell
$ git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
$ cd ws_livox
$ catkin_make


```

**最后配一下环境变量**

```shell
$ source ./devel/setup.sh
```

**以上就完成了运行fast-lio环境的配置**

## FAST-LIO源码编译

### **注：**

**①在编译fast-lio之前需要source一下livox_ros_driver的环境变量，否则编译不会通过**

**②如果想要使用自定义编译的PCL库，如要在环境变量中添加自定义编译的PCL库的路径：**

```shell
export PCL_ROOT={CUSTOM_PCL_PATH}
```

```SHELL
 $	cd ~/$A_ROS_DIR$/src
 $  git clone https://github.com/hku-mars/FAST_LIO.git
 $  cd FAST_LIO
 $  git submodule update --init
 $  cd ../..
 $  catkin_make
 $  source devel/setup.bash
```



### **使用ROSBAG数据集测试FAST-LIO**

**以livox_horizon为例**

**注意：如果需要录制bag文件，并且使用livox avia 或者 livox horizon 可以直接用：由于以上两款solid-state-lidar都有内置imu所以无需外参的标定**

```shell
$ roslaunch livox_ros_driver livox_lidar_msg.launch
$ rosbag record xxx.bag
```

**经测试fast-lio接受的livox雷达的点云消息不是传统的PointcCloud2类型，而是livox自定义的CustomMsg类型,IMU数据依旧是ros的sensor/Imu类型**

![](/home/arafat/Pictures/Screenshot 2022-08-18 19:53:11.png)

 **CustomMsg格式**

```
Header header             # ROS standard message header
uint64 timebase          # The time of first point
uint32 point_num      # Total number of pointclouds
uint8 lidar_id               # Lidar device id number
uint8[3] rsvd                 # Reserved use
CustomPoint[] points    # Pointcloud data
```

**上述自定义数据包中的自定义点云（CustomPoint）格式 :**

```
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

**另外还有pointcloud2(pcl::PointXYZI) 点云格式，请参考 PCL 库 point_types.hpp 文件中 the pcl::PointXYZI，pcl::PointXYZ 这是最简单的点的类型，存储着点的x,y,z信息。pcl::PointXYZI除此之外还包含了点的密集程度信息。**

**除此之外外还经常用到PointXYZINormal：两者相比较，PointXYZINormal包含除了点的x,y,z信息和密集程度信息之外，还有坐标(x,y,z)和曲率。**

**值得注意的是，在livox_ros_driver驱动包的launch目录只有livox_lidar_msg.launch文件发布的是CustomMsg类型的点云数据，其余的launch文件发布的都是PointCloud2类型的。**

**在fast-lio2中启动mapping_avia.launch或者mapping_horizon.launch（这两个launch文件是使用固态激光雷达）所以fast-lio2框架接收的固态激光雷达的点运数据一定是CustomMsg，当发布的消息的类型如果是pointcloud2类型时，会出现以下报错：lasermapping节点接收数据格式出错**

![](/home/arafat/Pictures/Screenshot 2022-08-18 20:21:16.png)



```shell
$ cd ~/$FAST_LIO_ROS_DIR$
$ source devel/setup.bash
$ roslaunch fast_lio mapping_horizon.launch
#以我自己录制的rosbag为例：
$ rosbag play balcony_5th_floor.bag
#如果需要实时建图 直接执行一下命令
#使用livox ros 驱动包之前 需要把驱动包source一下
$ roslaunch livox_ros_driver livox_lidar_msg.launch
```

### **使用livox_horizon实时slam**

#### **livox-horizon 激光雷达使用实录**

**浩界 Horizon 系列 Horizon 采用 Livox 自主研发的高速非重复扫描技术和自主设计的多线封装激光器，同等时间内的点云视场覆 盖率是 Mid 系列的三倍。随着积分时间的增大，点云视场覆盖率还会继续增大，探测到视场中的更多细节。**

**下面以 Horizon 在 0.1s 的扫描图案来说明点云分布。Horizon 在中间区域的扫描密度大，扫描线间隔平均约 0.2°（主要在 0.1°-0.3° 范围内），超过常见机械旋转式 64 线激光雷达。两边圆形区域的扫描密度比中间低，扫 描线间隔平均在 0.4°（主要在 0.2°-0.8° 范围内），0.1s 的综合扫描效果与常见机械旋转式 64 线激光雷达相当。 0.1s 时间内 Horizon 点云分布图如下：（坐标单位：度）**

![](/home/arafat/Pictures/Screenshot 2022-08-18 17:40:42.png)



**下图给出了不同积分时间下 Horizon 的视场覆盖率，和当前市场上常见的几款多线机械旋转式激光探测测距仪的对比。从图中可以看出，当积分时间小于 0.1s 时，Horizon 的视场覆盖率接近 60%，即与常见 64 线机械旋转式激光雷达相当；当积分时间继续增大，达到 0.5s 左右时，视场覆盖率将会接近 100%，即视场中几乎所有区域都会覆盖到。**

![](/home/arafat/Pictures/Screenshot 2022-08-18 17:44:00.png)

Horizon 系列点云参数如下表所示：

| **参数名称**          |                         **参数数值**                         |
| --------------------- | :----------------------------------------------------------: |
| **激光波长**          |                          **905nm**                           |
| **安全级别**          |            **Class 1(IEC 60825-1:2014) 人眼安全**            |
| **量程（@100klx）**   |     **90m@10% 反射率；130m@20% 反射率；260m@80% 反射率**     |
| **FOV**               |                **81.7°（水平)×25.1°（竖直)**                 |
| **距离随机误差**      |                     **（1σ@20m）< 2cm**                      |
| **角度随机误差**      |                         **1σ<0.05°**                         |
| **光束发散角度**      |                 **0.28°(竖直)×0.03°(水平)**                  |
| **数据率**            | **可配置第一回波或最强回波时：240,000 点/秒；双回波时：480,000 点/秒** |
| **虚警率（@100klx）** |                          **<0.01%**                          |



## **Livox Viewer**



**Livox Viewer是一款图形化操作的显示软件，有助于快速让你知道你的硬件连接和Ip配置有没有问题。**

**官方软件Livox Viewer仅支持Windows或ubuntu16.04/14.04,所以建议如果不是这两个版本有ros可以直接上。**

 **我使用的18.04也可以运行**

**链接: https://pan.baidu.com/s/1a-I6Db8Vmzhxd06x-eHJ8Q  密码: n1nh**
 **解压后进入解压文件，在终端中输入**

```shell
$ ./livox_viewer.sh
```

**这是中文官方文档，请根据以上文档完成硬件上的连接以及Ip配置，一切以官方文档为主：https://livox-wiki-cn.readthedocs.io/zh_CN/latest/index.html**

**以下是我本地的ip配置**

![](/home/arafat/Pictures/Screenshot 2022-08-18 18:57:11.png)

**这是livox_horizon中的参数信息以及静态ip配置**

![](/home/arafat/Pictures/Screenshot 2022-08-18 18:58:08.png)

![](/home/arafat/Pictures/Screenshot 2022-08-18 18:58:33.png)

**下图是livox viewer的显示界面**

![](/home/arafat/Pictures/Screenshot 2022-08-18 19:32:57.png)