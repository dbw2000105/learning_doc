# 多相机标定

## kalibr多相机标定

kalibr提供两种标定方法，标定板标定和IMU联合标定。IMU联合标定又分了两种，都需要移动摄像机测其加速度和角速度，这里只讨论第一种方法。

需要提供一个ros的bag和标定板参数文件作为输入，kalibr会同时输出所有相机的内参和外参。

==在使用相机进行标定的时候推荐使用低频率，建议4HZ。==

rosbag的获取方式有两种：

1.对于那些有ros driver的相机，直接订阅所有相机的话题即可。

2.对于没有ros drvier的相机，使用kalibr提供的bagcreater脚本生成rosbag

使用bagcreater的时候需要提供以下格式的文件

```
.bag文件的具体内容是：标定需要的图像数据。格式是：

+-- dataset-dir

+--cam0

│ +-- 1385030208726607500.png

│ +-- ...

│ \-- 1385030212176607500.png

+--cam1

│ +-- 1385030208726607500.png

│ +-- ...

│ \-- 1385030212176607500.png
```

文件格式是：19位时间戳（精确到ns）+.png

+ + .bag的制作工具：

```bash
kalibr_bagcreater--folder dataset-dir --output-bag awsome.bag
```

dataset-dir是数据输入路径：

其内文件结构应是这样:

```bash
/cam0/image_raw
/cam1/image_raw
```

awsome.bag 是制作好的bag文件。输出默认在kalibr_bagcreater同目录下。

Kalibr支持三种标定板，分别是Aprilgrid、Checkerboard和Circlegrid。

参数比较简单：见https://github.com/ethz-asl/kalibr/wiki/calibration-targets

运行标定

在制作完成标定需要文件后，就可以对cam进行标定了。

我们需要提供以下输入：

```
--bag filename.bag
 ROS bag containing the data
 --topics TOPIC_0 ... TOPIC_N
  list of all camera topics in the bag. matches the ordering of --models
 --models MODEL_0 ... MODEL_N
  list of camera/distortion models to be fitted. matches the ordering of --topics (see Supported models)
 --target target.yaml
  the calibration target configuration (see Calibration targets)
```

比如：你的标定板文件是checkerboard_7x6_50x50cm.yaml，图集文件是test.bag，执行

```bash
kalibr_calibrate_cameras --target checkerboard_7x6_50x50cm.yaml --bag test.bag --models pinhole-radtan pinhole-radtan --topics /cam0/image_raw /cam1/image_raw --show-extraction
```

如果传感器支持ROS系统，我们可以使用验证工具去验证标定的精度。