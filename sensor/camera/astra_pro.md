# Astra Pro 安装及使用（基于ROS）

## 驱动安装

2.1下载驱动及依赖

ubuntu16 kinetic及ubuntu18 melodic下载以下驱

OpenNI-Linux-x64-2.3.tar.bz2

```bash
$sudo apt-get install build-essential freeglut3 freeglut3-dev
$ldconfig -p | grep libudev.so.1
$cd /lib/x86_64-linux-gnu
$sudo ln -s libudev.so.x.x.x libudev.so.1
```

 

2.2编译驱动

 解压下载的OpenNI-Linux-x64-2.3驱动

```bash
$ cd OpenNI-Linux-x64-2.3
$ sudo chmod a+x install.sh

安装

$ sudo chmod a+x install.sh
$ sudo ./install.sh
```

插拔摄像头,source环境

$ source OpenNIDevEnvironment

```bash
编译

$ cd Samples/SimpleViewer
$ make

连接设备，执行示例

$ cd Bin/x64-Release
$ ./SimpleViewer 
```
## 下载astra pro 的ros安装包

可以从两个地方下载：

1、镜像网站：https://gitcode.net/mirrors/orbbec/ros_astra_camera?utm_source=csdn_github_accelerator

2、奥比中光官网下载 ==注意由于型号是Astra Pro，只能选OpenNI2的SDK==

![选区_001](/home/dbstg/Typora/我的文档/camera/astra_pro.assets/选区_001.png)

选择ROS下载即可，下载的包名字是**ros_astra_camera**

## 下载libuvc

ros_astra_camera依赖libuvc，需要从github上下载并编译源码

```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install 
sudo ldconfig
```

