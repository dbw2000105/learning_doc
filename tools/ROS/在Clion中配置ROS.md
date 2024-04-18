# 在Clion中配置ROS

配置网址：https://www.jetbrains.com/help/clion/ros-setup-tutorial.html

## 在ROS环境中启动Clion

首先在工作空间中将ROS添加到环境变量 source  /devel/setup.bash

启动clion：

sh /home/dbstg/app/CLion-2023.1.4/clion-2023.1.4/bin/clion.sh

或者直接输入clion

## 在Clion中打开ROS工程

1、在Clion中选择ROS工作空间下src文件夹里的CMakeLists.txt文件，之后选择 Open as Project

2、设置编译路径和cmake选项

Build directory： <WORKSPACE_DIRECTORY>/build

CMake options：-DCATKIN_DEVEL_PREFIX:PATH=<WORKSPACE_DIRECTORY>/devel

## 设置launch文件

在Eidtor中选择 file type，然后找到XML选项，在其中添加*.launch即可 