主要记录一些常见的常用的PCL库的函数。

# 点云格式的定义

一般会定义点的格式和点云的格式，使用using关键字为他们起别名，如下：

```c++
using PointType = pcl::PointXYZI; //定义点的格式 PointXYZI是最常用的点云格式，除此之外还有PointXYZRGB等
using PointCloudType = pcl::PointCloud<PointType>; //定义点云格式
using CloudPtr = PointCloudType::Ptr;//定义点云指针
```

# 其他函数

+ getVector3fMap()函数

`getVector3fMap()`是`pcl::PointXYZI`类的成员函数，它返回一个`Eigen::Map`对象，将点的`x`、`y`和`z`坐标映射到一个`Eigen::Vector3f`对象上。

```c++
PointType pt1；
pt1.getVector3fMap();//返回一个Eigen::Vector3f函数
```

# 匹配方法的使用

## ICP

在声明的时候可以直接声明，也可以声明成pcl的指针形式（Ptr），一般声明为Ptr格式
