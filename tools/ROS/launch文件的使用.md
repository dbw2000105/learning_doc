# ROS中launch的写法

格式：以<launch>开始，</launch>结束

以下是一个launch文件

```html

    <launch>

    <node pkg="turtlesim" name="turtle1" type="turtlesim_node"/>

    <node pkg="turtlesim" name="turtle1_key" type="turtle_teleop_key"/>

    </launch>
```

launch文件有**标签**和**属性**

```html
标签                                                                  
	☺<node> 启动一个节点

    ☺ <param> 设置参数服务器的参数

    ☺ <remap> 重映射

    ☺ <machine> 声明启动要使用的机器

    ☺ <include> 包含的launch文件 (嵌套)

    ☺ <group> 共享一个命名空间或者映射的元素组

    ☺ <args> 声明参数

```

==每个标签都有自己的属性，有的标签还有自己的子标签==

**需要注意的是，所有的标签都是launch的子标签**

对于含有子标签的标签，可以<> </>表达

## ==node 类型的使用==

node 定义了一个节点，其中里面包含了启动这个节点需要的参数

| node 节点参数（属性） | 含义 |
| :-----: | :---: |
| pkg="包名称" | 节点所属的包 |
| type=“nodeType” | 节点类型（节点的可执行文件名称） |
| name=“nodeName” | 节点名称（节点运行时的名称/初始化的节点名） |
| args="xxx xxx xxx"(可选) | 运行节点使用的参数（中间以空格隔开） |
| respawn="true" | false |
| respawn_delay="N"(可选) | 如果respawn为true，那么延迟N秒后启动节点 |
| ns="xxx"（可选） | 在指定命名空间xxx中启动节点 |
| output="log" | screen(可选) |

node 的子级标签

env 环境变量设置

remap 重映射节点名称

rosparam 参数设置

param 参数设置
注意，将参数写到节点内外是有区别的，写到节点内部则参数前有节点名作为其命名空间。如果要使用多层命名空间，例如在YAML文件中
group：
aaa:
   A:123
   B:456

## include 标签

其作用是将另一个xml格式的launch文件导入到当前文件

|          include 节点参数          |          含义          |
| :--------------------------------: | :--------------------: |
| file=“$(find 包名)/xxx/xxx.launch” |    要包含的文件路径    |
|          ns=“xxx” (可选)           | 在指定命名空间导入文件 |

子级标签

- env 环境变量设置
- arg 将参数传递给被包含的文件

## remap 标签

用于话题重命名。

|           remap 节点参数           |          含义          |
| :--------------------------------: | :--------------------: |
| file=“$(find 包名)/xxx/xxx.launch” |    要包含的文件路径    |
|          ns=“xxx” (可选)           | 在指定命名空间导入文件 |

## param标签

标签主要用于在参数服务器上设置参数，参数源可以在标签中通过 value 指定，也可以通过外部文件加载，在标签中时，相当于私有命名空间。 

|                param节点参数                |                          含义                          |
| :-----------------------------------------: | :----------------------------------------------------: |
|           name=“命名空间/参数名”            |                    可以包含命名空间                    |
|             value=“xxx” (可选)              |  定义参数值，如果此处省略，必须指定外部文件作为参数源  |
| type=“str、 int、double、bool、yaml” (可选) | 指定参数类型，如果未指定，roslaunch 会尝试确定参数类型 |

## rosparam 标签

此标签可以从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可以用来删除参数，标签在标签中时被视为私有。

|       rosparam节点参数        |           含义           |
| :---------------------------: | :----------------------: |
|         command="load         |           dump           |
| file=“$(find xxxxx)/xxx/yyy…” | 加载或导出到的 yaml 文件 |

## group 标签

标签可以对节点分组，具有 ns 属性，可以让节点归属某个命名空间

|   group节点参数    |     含义      |
| :----------------: | :-----------: |
| clear_params="true | false" (可选) |

子标签：

除了launch外其他所有的标签

## arg 标签

标签是用于动态传参，类似于函数的参数，可以增强launch文件的灵活性

**属性：**

- name="参数名称"
- default=“默认值”（）可选
- value=“数值”（可选）==不可与default并存==
- doc=“描述” 参数说明

arg示例

```html
<launch>
    <arg name="xxx" />
    <param name="param" value="$(arg xxx)" />
</launch>
```

**arg和param的区别：**

- `arg`虽然也是参数命令，但不储存在参数服务器中，不能提供给节点使用，只能在`launch`文件中使用，用来在运行中或直接在文件中修改`launch`文件中被`arg`定义的变量。

- `param`则是储存在参数服务器中，可以被节点使用，这是最本质的区别。

# rosbag写入launch

```xml
<node pkg="rosbag" type="record" 
        name="bag_record" 
        args="topicname1 topicname2 -o /home/xxx/bagfiles/aaa.bag"/>
```



# HTML文件的注释

使用这种方法对launch文件注释<!-- xxx -->

```html
<launch>  
     <!--include -->
     <node name="ORB_SLAM2" pkg="ORB_SLAM2" type="RGBD" />
  
</launch>
```

# 在launch文件中使用GDB调试

在launch文件中的节点内添加`launch-prefix="gdb -ex run --args"`即可启动GDB调试，其表示在运行launch的对话框内，显示bug信息。launch文件修改如下所示

```xml
<launch>
  <rosparam command="load" file="$(find livox_camera_calib)/config/calib.yaml" />
  <node 
    pkg="livox_camera_calib"
    type="lidar_camera_calib"
    name="lidar_camera_calib"
    output="screen"
    launch-prefix="gdb -ex run --args"  <!--此处添加代码-->
  />
  <node 
    pkg="rviz"
    type="rviz"
    name="rviz"
    args="-d $(find livox_camera_calib)/rviz_cfg/calib.rviz"
  />
</launch>

```

# 利用NodeHandle来进行launch文件中的数据获取

```cpp
  nh.param<string>("common/image_file", image_file, "");
  nh.param<string>("common/pcd_file", pcd_file, "");
  nh.param<string>("common/result_file", result_file, "");
  std::cout << "pcd_file path:" << pcd_file << std::endl;
  nh.param<vector<double>>("camera/camera_matrix", camera_matrix,
                           vector<double>());
  nh.param<vector<double>>("camera/dist_coeffs", dist_coeffs, vector<double>());
  nh.param<bool>("calib/use_rough_calib", use_rough_calib, false);
  nh.param<string>("calib/calib_config_file", calib_config_file, "");
```

其中，`nh.param`函数各部分的意思是

```cpp
nh.param<传入的参数类型>(launch文件中的参数名称, 程序中接受该参数的变量名称，该参数默认值）
```

