#  CMake语法

## 指定cmake的最小版本

```cmake
cmake_minimum_required(version 版本号)
```

例如：
```cmake
cmake_minimum_required(version 2.8)
```

## 定义工程名称

```cmake
#定义工程名称 
project(项目名称)
```

例如：

```cmake
project(gvins_estimater)
```

## 显示定义变量set

```cmake
set(var [value])
```

使用例程：

```cmake
# 第一种用法，生成代码文件列表
#先直接设置SRC_LIST的值
set(SRC_LIST add.h add.cpp)
#然后再在SRC_LIST中追加main.cpp 
set(SRC_LIST ${SRC_LIST} main.cpp)

# 第二中用法，设置库生成目录或者可执行文件生成目录
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib/linux) 
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
```

## 动态库和静态库的编译

```cmake
# 编译静态库
add_library(库名称 STATIC 代码文件名称) 

# 编译动态库
add_library(库名称 SHARED 代码文件名称) 

# 编译可执行程序
add_executable(可执行程序名 代码文件名称)
```

> + 静态库（.a）：程序在编译链接的时候把库的代码链接到可执行文件中。程序运行的时候将不再需要静态库
> + 动态库（.so）：程序在运行的时候才去链接动态库的代码，多个程序共享使用库的代码。
> + 一个与动态库链接的可执行文件仅仅包含它用到的函数入口地址的一个表，而不是外部函数所在目标文件的整个机器码
> + 在可执行文件开始运行以前，外部函数的机器码由操作系统从磁盘上的该动态库中复制到内存中，这个过程称为动态链接（dynamic linking）
> + 动态库可以在多个程序间共享，所以动态链接使得可执行文件更小，节省了磁盘空间。操作系统采用虚拟内存机制允许物理内存中的一份动态库被要用到该库的所有进程共用，节省了内存和磁盘空间。
>   

![20150305225040244](./cmake.assets/20150305225040244.png)

```cmake
# 编译静态库
add_library(add STATIC add.h add.cpp)
add_library(add STATIC ${ADD_SRC} ${ADD_HDR})

# 编译动态库
add_library(add  SHARED add.h add.cpp) 
add_library(add SHARED  ${ADD_SRC} ${ADD_HDR})

# 编译可执行程序
add_executable(main add.h add.cpp mai.cpp)
add_executable(main ${MAIN_SRC} ${MAIN_HDR})

```

## 指定静态库或者动态库编译输出目录

例如将当前编译的静态库或者动态库输出到当前项目文件夹lib子目录下

```cmake
set(LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)
```

##  指定可执行程序编译输出目录

例如将当前可执行程序输出到当前项目文件夹的bin子目录下

```cmake
#设定可执行二进制文件的目录
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

```

## 设置静态链接库搜索目录

例如将链接库搜索目录设置为当前项目文件夹下lib/linux文件夹

```c++
link_directories( ${PROJECT_SOURCE_DIR}/lib/linux)
```

## 链接静态库

```cmake
link_libraries(
	静态库1
	静态库2
	静态库3
	...
)

```

注意，`link_libraries`中的静态库为全路径，常与`1.7 link_directories` 搭配使用，例如：

`lib1.a lib2.a`在目录`${PROJECT_SOURCE_DIR}/lib/linux`下，则先设置链接目录，再链接相应的库

```cmake
#设置链接目录
link_directories( ${PROJECT_SOURCE_DIR}/lib/linux)
link_libraries(
		lib1.a
		lib2.a
)
```

## 链接动态库

```cmake
target_link_libraries(所需生成的文件名称 所需链接的动态库名称)
```

例如

```cmake
target_link_libraries(main dl)
```

##  link_libraries 和 target_link_libraries 区别

在`cmake`语法中，`link_libraries`和`target_link_libraries`是很重要的两个链接库的方式，虽然写法上很相似，但是功能上有很大区别：

> (1) link_libraries用在add_executable之前，target_link_libraries用在add_executable之后
> (2) link_libraries用来链接静态库，target_link_libraries用来链接导入库，即按照header file + .lib + .dll方式隐式调用动态库的.lib库

## file语法

### 将文件夹所有的类型的文件添加到文件列表

例如将当前文件夹下所有`.cpp`文件的文件名加入到`MAIN_SRC`中，将当前文件夹下所有`.h`加入到`MAIN_HDR`中。

```cmake
file(GLOB MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB MAIN_HDR ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

```

### 递归搜索该文件夹，将文件夹下（包含子目录）符合类型的文件添加到文件列表

例如将当前文件夹下（包括子目录下）所有.cpp文件的文件名加入到MAIN_SRC中，所有.h加入到MAIN_HDR中

```cmake
file(GLOB_RECURSE MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE MAIN_HDR ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

```

### 查找目录下所有源文件并添加到变量

使用aux_source_directory(文件路径 变量)方法

```cmake
# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

```

## List操作

###  List移除指定项

```cmake
list(REMOVE_ITEM MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/add.cpp)

```

### 将两个List链接起来

```cmake
# 搜索当前目录
file(GLOB  MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB  MAIN_HDR ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

# 递归搜索当前目录下src子目录
file(GLOB_RECURSE MAIN_SRC_ELSE  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE MAIN_HDR_ELSE  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

# 将MAIN_SRC_ELSE中的值添加到MAIN_SRC 
# 将MAIN_HDR_ELSE中的值添加到MAIN_HDR 
list(APPEND MAIN_SRC ${MAIN_SRC_ELSE})
list(APPEND MAIN_HDR ${MAIN_HDR_ELSE})

```

## include_directories

是用来提供找头文件路径的，打个比方，我现在想要`#include"cv.h"`,但是这个`cv.h`的路径是`/usr/local/include/opencv,`那么我总不能在主函数头前写#`include “/usr/local/include/opencv/cv.h“`吧，这个时候就用到`include_directories`了，它提供了一个搜索头文件暂时的根目录，即你可以在`cmakelists`中写上`include_directories(/usr/local/include)`来让库文件搜索以`/usr/local/include`为基础，即在`main`函数前写上`#include “opencv/cv.h"`即可
比如加入`OpenCV`的`include`

```cmake
include_directories( ${OpenCV_INCLUDE_DIRS} )
```



## 添加子文件夹

例如

```cmake
add_subdirectory(src)
```

该语句会在执行完当前文件夹`CMakeLists.txt`之后执行`src`子目录下的`CMakeLists.txt`

## message输出消息机制

输出正常：

```cmake
message(STATUS "Enter cmake ${CMAKE_CURRENT_LIST_DIR}")
```

输出警告

```cmake
message(WARNING "Enter cmake ${CMAKE_CURRENT_LIST_DIR}")
```

输出错误：

```cmake
message(FATAL_ERROR "Enter cmake ${CMAKE_CURRENT_LIST_DIR}")
```



## 设置编译选项

设置编译选项可以通过add_compile_options命令，也可以通过set命令修改CMAKE_CXX_FLAGS或CMAKE_C_FLAGS。
 方式1：

```cmake
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -frtti -fpermissive -fexceptions -pthread")
```

方式2：

```cmake
add_compile_options(-march=native -O3 -fexceptions -pthread -fPIC)

```

这两种方式的区别在于：

> `add_compile_options`命令添加的编译选项是针对所有编译器的(包括c和c++编译器)，而`set`命令设置`CMAKE_C_FLAGS`或`CMAKE_CXX_FLAGS`变量则是分别只针对c和c++编译器的。

## 预定义变量
### 基本变量

`PROJECT_SOURCE_DIR-`----------------------------------------我们使用`cmake`命令后紧跟的目录，一般是工程的根目录；
`PROJECT_BINARY_DIR` ------------------------------------------执行`cmake`命令的目录,通常是`\${PROJECT_SOURCE_DIR}/build`；
`CMAKE_INCLUDE_PATH`-----------------------------------------系统环境变量,非`cmake`变量；
`CMAKE_LIBRARY_PATH`------------------------------------------系统环境变量,非`cmake`变量；
`CMAKE_CURRENT_SOURCE_DIR`---------------------------当前处理的CMakeLists.txt所在的路径；
`CMAKE_CURRENT_BINARY_DIR`-----------------------------target编译目录（使用`ADD_SURDIRECTORY(src bin)`可以更改此变量的值 `SET(EXECUTABLE_OUTPUT_PATH` <新路径>)并不会对此变量有影响,只是改变了最终目标文件的存储路径）；
`CMAKE_CURRENT_LIST_FILE`--------------------------------输出调用这个变量的`CMakeLists.txt`的完整路径；
`CMAKE_CURRENT_LIST_LINE`--------------------------------输出这个变量所在的行；
`CMAKE_MODULE_PATH`-----------------------------------------定义自己的`cmake`模块所在的路径（这个变量用于定义自己的`cmake`模块所在的路径，如果你的工程比较复杂，有可能自己编写一些`cmake`模块，比如`SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake),`然后可以用`INCLUDE`命令来调用自己的模块）；
`EXECUTABLE_OUTPUT_PATH`------------------------------重新定义目标二进制可执行文件的存放位置；
`LIBRARY_OUTPUT_PATH`--------------------------------------重新定义目标链接库文件的存放位置；
`PROJECT_NAME`-------------------------------------------------返回通过`PROJECT`指令定义的项目名称；

### 开关选项

`BUILD_SHARED_LIBS`---------------------------------------------控制默认的库编译方式。如果未进行设置,使用ADD_LIBRARY时又没有指定库类型,默认编译生成的库都是静态库；
`CMAKE_C_FLAGS`-------------------------------------------------设置`C`编译选项，也可以通过指令`ADD_DEFINITIONS()`添加；
`CMAKE_CXX_FLAGS`----------------------------------------------设置C++编译选项，也可以通过指令`ADD_DEFINITIONS()`添加；
`CMAKE_C_COMPILER`--------------------------------------------指定`C`编译器；
`CMAKE_CXX_COMPILER`----------------------------------------指定`C++`编译器；
`CMAKE_BUILD_TYPE`:：`build` 类型(`Debug`, `Release`, …)`-CMAKE_BUILD_TYPE=Debug`

```cmake
# -fPIC：生成动态库，-fopenmp 开启多线程，-O3 对代码进行优化，-g 打印调试信息，-Wall 打印所有警告信息, pthread 支持多线程
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++17 -g -O3 -fopenmp -pthread")
```



##  自动检测编译器是否支持C++11

```cmake
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
```

## find_package

命令：

```cmake
find_package(<PackageName> [version] [EXACT] [QUIET] [REQUIRED])
```

`version:` 版本合适（大版本号相同）
 `EXACT:` 版本必须一致
 `QUIET`: 没找到包也不会报错
 `REQUIRED`: 必须找到该包，否则停止

**find_package采用两种模式搜索库：**

1. `Module`模式：搜索`CMAKE_MODULE_PATH`指定路径下的FindXXX.cmake文件，执行该文件从而找到`XXX`库。其中，具体查找库并给`XXX_INCLUDE_DIRS`和`XXX_LIBRARIES`两个变量赋值的操作由`FindXXX.cmake`模块完成（先搜索当前项目里面的Module文件夹里面提供的`FindXXX.cmake`，然后再搜索系统路径`/usr/local/share/cmake-x.y/Modules/FindXXX.cmake`）
2. `Config`模式：搜索`XXX_DIR`指定路径下的`XXXConfig.cmake`文件，执行该文件从而找到`XXX`库。其中具体查找库并给`XXX_INCLUDE_DIRS`和`XXX_LIBRARIES`两个变量赋值的操作由`XXXConfig.cmake`模块完成。比如我的系统是`ubuntu20`，对于`cv_bridge`的支持没有那么好，那么我就需要自己编译这个模块，需要设置查找路径才能找到我自己编译的`cv_bridge`，`set(cv_bridge_DIR /home/jianing/package/catkin_ws/devel/share/cv_bridge/cmake)`
3. 对于可能没有***.cmake和***Config.cmake的库文件，可以直接找到其头文件和库文件所在文件夹，直接进行路径赋值：
   `SET(LAPACK_INCLUDE_DIRS /usr/local/include)` `SET(LAPACK_LIBRARIES /usr/local/lib)`

两种模式看起来似乎差不多，不过`cmake`默认采取`Module`模式，如果`Module`模式未找到库，才会采取`Config`模式。如果XXX_DIR路径下找不到`XXXConfig.cmake`文件，则会找`/usr/local/lib/cmake/XXX/`中的XXXConfig.cmake文件。总之，`Config`模式是一个备选策略。通常，库安装时会拷贝一份`XXXConfig.cmake`到系统目录中，因此在没有显式指定搜索路径时也可以顺利找到。

如果找到这个包，则可以通过在工程的顶层目录中的`CMakeLists.txt` 文件添加 `include_directories(XXX_INCLUDE_DIRS)` 来包含库的头文件，添加target_link_libraries(XXX_LIBRARIES)命令将源文件与库文件链接起来。
### find_package的查询路径：

#### 环境变量：

首先需要知道本机的环境变量，可以使用`export`命令显示所有环境变量：

```bash
export
```

也可以使用`echo`命令显示某个环境变量，比如`PATH`：

```bash
echo $PATH
```

#### 设定查询路径

设定查询路径通过`cmake`中的`CMAKE_MODULE_PATH`关键字设置寻找.`cmake`的位置：

```cmake
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
```

上面的指令把工程根目录下的`cmake`文件夹添加为`.cmake`文件搜索路径，是优先搜索的路径。

另外，还可以直接设置某个包的`.cmake`位置：

```cmake
set(OpenCV_DIR /path_to_opencv)
find_package(OpenCV)
```

上面的指令使`find_package()`寻找`OpenCV`时，最优先查找`/path_to_opencv`路径下的`.cmake`文件。

#### 多重版本库更改实例（ceres）

R2LIVE中用的ceres是低版本的(1.14)，而本机中安装的是ceres 2.0.0 在链接的时候会出现 **error: ‘integer_sequence’ is not a member of ‘std’** 等一系列报错，是因为新版本的ceres不再使用一些老的方法。

因为不想替换掉本机中的ceres版本，在不考虑使用docker的情况下，尝试使用多版本ceres。

+ 从github上下载低版本ceres(1.14)。

  cmake -DCMAKE_INSTALL_PREFIX=/home/dbstg/clibraries/ceres-solver-1.14.0/cmake_install ..

  使用 -DCMAKE_INSTALL_PREFIX是指定库的安装路径，即make install 的路径，默认会安装在/usr/local/include中，但是我们不想覆盖掉原本的高版本ceres，故在低版本ceres工程下新建一个cmake_install 文件夹，在make install 时就会把所有安装文件全部放在cmake_install 中

+ make -j8
+ make install

+ 最后需要在所有使用ceres的CMakeLists.txt文件中指定ceres的版本和路径。

```cmake
set(Ceres_DIR "/home/dbstg/clibraries/ceres-solver-1.14.0/cmake_install/lib/cmake/Ceres")
find_package(Ceres 1.14.0 REQUIRED)
```

#### 默认路径

如果没有设定查询路径，或者在设定查询路径没有找到合适的`.cmake`时，`cmake`继续在默认查询路径中寻找`.cmake`文件，这些默认查询路径有：

```cmake
PATH
CMAKE_PREFIX_PATH
CMAKE_FRAMEWORK_PATH
CMAKE_APPBUNDLE_PATH
```

如果`PATH`路径为`/bin`或/sbin文件夹，则从上一级目录查找。

以默认路径为根目录，`cmake`将检查根目录下的`/lib/cmake,/lib/<arch>/cmake,/share/cmake`下寻找`.cmake`文件，根据`.cmake`生成对应的头文件目录和库文件路径。
### 常用的find_package

#### 调用OpenCV

```cmake
find_package(OpenCV 3.4.5 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME}   ${OpenCV_LIBS}}

//或者
find_package(OpenCV 3.4.5 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
```

#### 调用Ceres

```cmake
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${CERES_LIBRARIES})
```

#### 调用Boost

```cmake
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
```

#### 调用Eigen

```cmake
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
```

Eigen最神奇的是并不需要链接动态库或者静态库，只需要引用它的头文件就可以用了

#### 调用PCL

```cmake
find_package(PCL 1.3 REQUIRED COMPONENTS common io)     
include_directories(${PCL_INCLUDE_DIRS})                
link_directories(${PCL_LIBRARY_DIRS}) //这个不知道要不要写，建议先不写，如果编译或者运行出错的时候，可以在考虑一下            add_definitions(${PCL_DEFINITIONS})                     
add_executable(${PROJECT_NAME} xxxx)            
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})  
```

#### 调用Pangolin

```cmake
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} ）
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES}）
```

#### 调用realsense2

```cmake
find_package(realsense2)
# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()
```

#### 调用sophus

```cmake
find_package(Sophus REQUIRED )
message("Sophus dir ${Sophus_INCLUDE_DIRS}")
message("Sophus lib ${Sophus_LIBRARIES}")
include_directories(${Sophus_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${Sophus_LIBRARIES})
```

#### 调用GTSAM

```cmake
find_package(GTSAM REQUIRED)
include_directories(
  ${GTSAM_INCLUDE_DIR}）
link_directories(
  	${GTSAM_LIBRARY_DIRS}
)
target_link_libraries(${PROJECT_NAME} gtsam)

```

#### 调用yaml-cpp

```cmake
find_package(yaml-cpp REQUIRED)
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_INCLUDE_DIR})
```

## catkin_package

我们在一些`ros`功能包的`CMakeLists`文件中经常可以看见`catkin_package`

```cmake
catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)
```

但是通过`find_package()`我们已经完成找头文件和库的目的了，那么`catkin_package()`要做什么呢

`catkin_package()`是`catkin`提供的`CMake`宏，用于为`catkin`提供构建、生成`pkg-config`和`CMake`文件所需要的信息。

有五个参数可选：

`INCLUDE_DIRS `- 声明给其它`package`的`include`路径

`LIBRARIES` - 声明给其它`package`的库

`CATKIN_DEPENDS` - 本包依赖的`catkin package`

`DEPENDS `- 本包依赖的非`catkin package`

`CFG_EXTRAS` - 其它配置参数

意思就是如果其他功能包使用本功能包的话

```cmake
find_package(catkin REQUIRED
  COMPONENTS)

catkin_INCLUDE_DIRS  catkin_LIBRARIES
```

当中会包含我们在本功能包声明的`include`路径和库，对于`DEPENDS`依赖项，会将`DEPENDS`的头文件路径和库添加到本功能包下的`include`路径和库。

举个例子就是

```cmake
catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
  DEPENDS Boost
)
```

当前我的`DEPENS`包含了`Boost`库，当其他功能包调用这个功能包也需要`Boost`库的时候，就可以不需要再`find_package(Boost)`

但是如果功能包明确需要使用库的话，仍然建议显式寻找库`find_package(Boost)`，而不是通过包含其他功能包隐式寻找，这么做的目的是避免其他功能包修改依赖项导致功能包编译失败。
