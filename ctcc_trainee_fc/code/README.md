## 基于相机和激光雷达融合的三维目标检测

###　一、简介

这个仓主要是利用路测杆的激光雷达点云和图像信息，进行障碍物检测与跟踪，同时建立了二维的珊格地图。

主要分为三个步骤

1. sensor_processing

这个package主要功能为获取激光雷达点云和相机数据，来生成带语意信息的点云，同时还会生成珊格地图

2. detection

主要功能为使用DBSCAN聚类对detection_grid话题进行处理，获取目标信息。

3. tracking
使用ukf对目标进行跟踪

### 二、安装

主要环境：
```
CUDA10.2
cudnn8.0.4
OPENCV3.4
libtorch1.6.0
```

下载libtorch
```zsh
cd src/object_tracking/sensor_processing
wget https://download.pytorch.org/libtorch/cu102/libtorch-cxx11-abi-shared-with-deps-1.6.0.zip
unzip libtorch-cxx11-abi-shared-with-deps-1.6.0.zip
cd libtorch/
sudo cp -r lib/* /usr/lib
```

下载权重文件：
下载地址：
将权重文件放入`srcsrc/object_tracking/sensor_processing/weight`中

编译运行：
```zsh
catkin_make_isolated --install
source ./install_isolated/setup.zsh
roslaunch percetion.launch
```

### 三、结果

<center>

<img src="./image/result.png" alt="result" width="100%" height="50%" align="result" />

</center>