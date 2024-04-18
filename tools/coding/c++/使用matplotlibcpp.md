# CMakeLists.txt编写

```cmake
#python
find_package(Python REQUIRED COMPONENTS Development)
include_directories(${Python_INCLUDE_DIRS})

# matplotlibcpp
include_directories(/home/dbstg/clibraries/matplotlib-cpp-master)
add_library(matplotlibcpp INTERFACE)
target_sources(matplotlibcpp INTERFACE /home/dbstg/clibraries/matplotlib-cpp-master/matplotlibcpp.h)
```

注意：确保已经成功安装matplotlibcpp源码

