# ROS 与Eigen

## tf::vectorMsgToEigen()

用于将 ROS 中的 `geometry_msgs::Vector3` 类型的向量转换为 Eigen 中的 `Eigen::Vector3d` 类型的向量。

`tf::vectorMsgToEigen()` 函数有一个参数，即要转换的 `geometry_msgs::Vector3` 类型的向量。它返回一个 `Eigen::Vector3d` 类型的向量，表示转换后的向量。

以下是 `tf::vectorMsgToEigen()` 函数的使用示例：

```c++
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Core>

geometry_msgs::Vector3 ros_vec;
ros_vec.x = 1.0;
ros_vec.y = 2.0;
ros_vec.z = 3.0;

Eigen::Vector3d eigen_vec = tf::vectorMsgToEigen(ros_vec);
```

在上面的示例中，首先创建了一个 `geometry_msgs::Vector3` 类型的向量 `ros_vec`，并将其 x、y、z 分量分别设置为 1.0、2.0、3.0。然后使用 `tf::vectorMsgToEigen()` 函数将 `ros_vec` 转换为 `Eigen::Vector3d` 类型的向量 `eigen_vec`。

需要注意的是，`tf::vectorMsgToEigen()` 函数只能用于将 `geometry_msgs::Vector3` 类型的向量转换为 `Eigen::Vector3d` 类型的向量。如果要将其他类型的向量转换为 `Eigen::Vector3d` 类型的向量，可以使用 `Eigen::Map` 类或 `Eigen::Vector3d::Map()` 函数。

## tf::vectorEigenToMsg()

将Eigen::Vector3d转换为geometry_msgs::Vector3

`tf::vectorEigenToMsg()` 函数有两个参数，第一个参数是要转换的 `Eigen::Vector3d` 类型的向量，第二个参数是要存储转换结果的 `geometry_msgs::Vector3` 类型的向量。

```c++
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Core>

Eigen::Vector3d eigen_vec(1.0, 2.0, 3.0);
geometry_msgs::Vector3 ros_vec;

tf::vectorEigenToMsg(eigen_vec, ros_vec);
```

在上面的示例中，首先创建了一个 `Eigen::Vector3d` 类型的向量 `eigen_vec`，并将其 x、y、z 分量分别设置为 1.0、2.0、3.0。然后使用 `tf::vectorEigenToMsg()` 函数将 `eigen_vec` 转换为 `geometry_msgs::Vector3` 类型的向量 `ros_vec`。

需要注意的是，`tf::vectorEigenToMsg()` 函数只能用于将 `Eigen::Vector3d` 类型的向量转换为 `geometry_msgs::Vector3` 类型的向量。如果要将其他类型的向量转换为 `geometry_msgs::Vector3` 类型的向量，可以使用 `geometry_msgs::Vector3` 类的构造函数或者 `geometry_msgs::Vector3Stamped` 类的 `vector` 成员变量。

# ROS与CV

一般我们借助cvbridge完成转换

## ROS2CV

```c++
typedef boost::shared_ptr<CvImage> CvImagePtr;
typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

//方法1：拷贝法，即如果我们要修改数据，我们必须复制一份ros的信息数据
CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                    const std::string& encoding = std::string());
//方法2：共享法，即如果我们不修改数据。我们可以安全地分享由ros消息所拥有的数据，而不用复制。
CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                          const std::string& encoding = std::string());

```

## CV2ROS

```c++
sensor_msgs::ImagePtr  msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", Right_raw).toImageMsg();
```

# ROS与PCL

三种常用点云数据格式：

- pcl::PointCloud< PointT>
- pcl::PCLPointCloud2
- snesor_msgs::PointCloud2

## ROS2PCL

+ sensor_msgs::PointCloud2转pcl::PCLPointCloud2

```c++
pcl_conversion::toPCl(sensor_msgs::PointCloud2,pcl::PCLPointCloud2)
```

+ sensor_msgs::PointCloud2转pcl::PointCloud< PointT>

```c++
pcl::fromROSMsg(sensor_msgs::PointCloud2,pcl::PointCloud<PointT>)
```

## PCL2ROS

+ pcl::PCLPointCloud2转sensor_msgs::PointCloud2

```c++
pcl_conversion::fromPCL(pcl::PCLPointCloud2,sensor_msgs::PointCloud2)
```

+ pcl::PointCloud< PointT>转 sensor_msgs::PointCloud2

```c++
pcl::toROSMsg(pcl::PointCloud<PointT>,snesor_msgs::PointCloud2)
```

## PCL2PCL

+ pcl::PCLPointCloud2转pcl::PointCloud< PointT>

```c++
pcl::fromPCLPointCloud2(pcl::PCLPointCloud2,pcl::PointCloud<PointT>
```

+ pcl::PointCloud< PointT>转 pcl::PCLPointCloud2

```c++
pcl::toPCLPointCloud2(pcl::PointCloud< PointT>,pcl::PCLPointCloud2)
```

