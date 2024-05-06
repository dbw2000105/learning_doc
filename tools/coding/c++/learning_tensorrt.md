# tensorRT部署指南

## tensorRT安装

+ **准备工作**

首先需要确定cuda和tensorRT的版本对应，本机CUDA 11.8 对应的tensorRT版本8.6 GA，下载tar的版本。[下载地址](https://developer.nvidia.com/nvidia-tensorrt-8x-download)

**note：如果cuda的版本高于12.1，那么就要下载tensorRT 9 以上的版本了。**

需要下载cuda版本对应的cudnn，这个去网上查一下对应的版本下载即可，我这里直接下载了cudnn的deb格式(安装比较方便)

+ 配置tensorRT环境

首先确保有一个专门的conda环境，我这里是为了配置yolov8创建的python 3.8.10版本。

在这个环境下进入tensorRT文件夹，找到python文件夹，pip安装对应的whl

**note：cp后面的数字对应python的版本**

在zshrc下配置相关的环境变量:

```bash
export TENSORRT_DIR=/home/dbstg/clibraries/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8/TensorRT-8.6.1.6

export LD_LIBRARY_PATH=$TENSORRT_DIR/lib:$LD_LIBRARY_PATH

export PATH=/home/dbstg/clibraries/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8/TensorRT-8.6.1.6/bin:$PATH
```

配置完后，刷新环境变量，可以输入

```bash
echo LD_LIBRARY_PATH
```

检查tensor的环境变量是否被加进去了。

如果上述全部配置成功，可以在python中进行检查