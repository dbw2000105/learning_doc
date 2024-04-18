# SDK基础介绍

OpenNI最核心的四个类：OpenNI、Device、VideoStream和VideoStreamRef。下面依次解释他们的作用。

##**OpenNI**：       

OpenNI  类是API的静态入口，每一个OpenNI2.0的应用程序都需要使用这个类来初始化SDK以及驱动，以便可以创建合法的设备对象。OpenNI类还定义了一个**监听器类**以及相应的事件，使得当发生设备连接，设备连接断开及设备配置改变时，应用程序能得到事件通知。另外，OpenNI类提供了一个函数用来获取OpenNI的版本信息，提供了一个函数用来等待从列表中任何一个流产生的数据帧。

##**Device**

Device类是对一个特定设备的抽象，特定的设备或者是一个硬件设备，或者是从一个硬件设备记录下来的文件设备。该类提供了**连接一个设备，以及获取设备的配置信息和设备支持的流**的种类的能力。该类提供了方法来查询和修改设备的配置参数，包括**启动深度和彩色流以及帧同步**等。当创建和初始化VideoStream类时会使用Device类.为了调用VideoStream.create()功能，我们需要一个Device对象的指针来作为VideoStream.create()函数的其中一个参数，对于应用开发者来说，将Device用来创建及初始化VideoStream是一个最基本的使用Device类的场景。在创建设备对象之前，OpenNI::initialize()函数必须已经被调用，使得API能够获得系统中的设备驱动。

##**VideoStream**

视频流是来自特定数据源的帧的顺序流。想象一下一卷老式电影胶片，其中，视频将作为单个图像快照依次记录在长条胶片上。每个图像快照便是一帧，您可以将整部影片想象成一个流，而**流与电影的区别在于流不一定有确切的终点**。

VideoStream 对象封装了设备上的一个单独的视频流，VideoStream  对象创建之后，**可以用来启动设备上的数据流和从设备上读取数据帧**。VideoStream类是OpenNI中获取数据的核心,  VideoStream类提供了手动**以循环方式**读取数据的能力以及提供了**以事件驱动方式**获取数据的事件类及监听器类的定义。

除了获取数据帧的接口，VideoStream类提供了一系列函数用来获取一个VideoStream对象的信息，比如视场、支持的视频模式以及支持的最大及最小像素值。除了获取数据，VideoStream对象还用来配置一个指定流的属性，特别地，可以用来控制裁剪、镜像和视频模式。

创建流时，我们需要 一个指向合法的已初始化设备（该设备需支持待创建的流类型）的指针作为参数。同一个传感器上可以创建多个视频流，这对当一个应用的多个模块都需要单独读取帧数据时很有用。

##**VideoStreamRef**

VideoFrameRef 类是对**一个视频帧及其元始数据**的抽象。视频帧是一个特定的视频流在某个时间的输出。输出的数据中包含单个帧（Color、IR或者Depth）以及对应的元数据。

一个VideoFrameRef类的对象==并不是真正持有帧中的数据，而只是帧的引用==。这个引用可以通过销毁VideoFrameRef对象或者通过调用release()方法来释放。当帧的最后一个引用释放后，帧中的数据才会被真正的释放。

最常用的获取VideoFrameRef对象的方法是调用VideoStream::readFrame()。

VideoFrameRef对象引用的数据以像素数组格式存储。每个像素的类型都与指定的像素格式保持一致。

# 获取流数据

## 数据流类型

OpenNI2 SDK支持三种数据流类型，分别是彩色流、深度流和红外流。这些数据流由传感器生成，并通过SDK传递给应用程序。

| 流类型       | 描述                                                         |
| ------------ | :----------------------------------------------------------- |
| SENSOR_COLOR | 来自传感器的RGB像素数据。每个彩色帧中包含了每个像素的每个颜色分量的0-255范围内的值。 |
| SENSOR_DEPTH | 来自传感器的深度数据。每个深度帧中包含了传感器视场内的每个像素的距离值（默认情况下的单位：==毫米==）。 |
| SENSOR_IR    | 来自传感器的红外数据。                                       |

## 获取数据

OpenNI2 SDK提供了两种方法来获取流数据。根据特定用例和应用程序的复杂性，其中的一种方法可能比另一种更适合。

###**轮询**

获取帧数据的轮询方法是获取流数据的最直接方法。为使用此方法，您只需要调用OpenNI::waitForAnyStream并通过VideoStream::readFrame()方法读取流数据。如果有新的数据产生了，readFrame()方法就会提供一个可以访问最新产生的视频帧的帧引用（VideoFrameRef）。如果没有新的帧产生，OpenNI::waitForAnyStream这个方法就会阻塞直到有新的帧产生。如果您想限制SDK以等待新帧到达的时间，可以将超时时间作为参数传递给OpenNI::waitForAnyStream函数。

```c++
//核心代码
int changedStreamDummy;
VideoStream* pStream = &depth;

//轮询方式核心函数
rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT); 
if (rc != STATUS_OK)//等待失败
{
    printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
    return 5;
}
//查到新的帧进来，调用readFrame读取帧
rc = depth.readFrame(&frame);
if (rc != STATUS_OK)
{
    printf("Read failed!\n%s\n", OpenNI::getExtendedError());
    return 6;
}
```

### 事件

使用基于事件的方法获取帧数据需要少量的附加设置，但允许开发人员将帧的处理委托给一个或多个单独的类。OpenNI2  SDK提供了一个名为VideoStream::NewFrameListener的抽象类，该类只实现了一个名为NewFrameListener::onNewFrame的函数。一旦VideoStream流中有数据帧到来并准备好被处理时，就会立即调用NewFrameListener::onNewFrame函数。

从NewFrameListener中派生的侦听器类的示例：

```c++
class PrintCallback : public VideoStream::NewFrameListener
{
public:
    void onNewFrame(VideoStream& stream)
    {
        stream.readFrame(&m_frame);
    }
private:
    VideoFrameRef m_frame;
};
```

定义了侦听器类后，为了使用它，必须在应用程序中实例化侦听器，然后使用VideoStream::addNewFrameListener函数将其添加到 VideoStream中。

```c++
//核心代码
PrintCallback depthPrinter; //侦听器实例化

// Register to new frame
depth.addNewFrameListener(&depthPrinter);

// Wait while we're getting frames through the printer
while (true)
{
    Sleep(100);
}
```

实际上，上面代码中的while循环不需要一直执行，当应用程序关闭或发生另一个特定于应用程序的事件发生时，需要退出循环。

# 遇到的问题

## 不能找到设备

刚码几行代码就出现了这样的问题：

![image-20221211192838963](/home/dbstg/Typora/我的文档/camera/OpenNI2学习手册.assets/image-20221211192838963.png)

这个问题是由以下代码引起的

```c++
openni::Device device; //实例化 设备类
//open device
    rc = device.open(openni::ANY_DEVICE); //打开设备
    if (rc != openni::STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", oniGetExtendedError());
        return 2;
    }
```

==解决问题：==

报找不到设备的错误是因为OpenNI2调用Astra Pro需要使用它的驱动，而在本工程中没有引入驱动文件。

官网给出的是makefile文件的配置方法，但是不是cmake。

![image-20221211212352319](/home/dbstg/Typora/我的文档/camera/OpenNI2学习手册.assets/image-20221211212352319.png)

由上图可知，需要3个.so文件，将这三个文件的路径加入cmake即可解决问题

```cmake
link_directories(/home/dbstg/slam/astra/OpenNI-Linux-x64-2.3.0.66/Redist/OpenNI2/Drivers)
link_directories(/home/dbstg/slam/astra/OpenNI-Linux-x64-2.3.0.66/Redist)
```

参考过的一些网站：

1、https://stackoverflow.com/questions/48916317/orbbec-and-openni2-deviceopen-using-default-no-devices-found

2、https://developer.orbbec.com.cn/technical_library.html?id=30