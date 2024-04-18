# filesystem 文件操作系统

什么是boost::filesystem?

> - 目录、文件处理是脚本语言如shell、python所擅长的领域，C++语言缺乏对操作系统文件的查询和操作能力，因此C++程序员经常需要再掌握另外一门脚本语言以方便自己的工作，这增加了学习成本
> - filesystem是一个可移植的文件系统操作符，已被收入C++17版本。它在底层做了大量的工作，使用了POSIX标准表示文件系统的路径，接口很类似标准库的容器和迭代器。使C++具有了类似脚本语言的功能，可以跨平台操作目录、文件，写出通用的脚本程序
> - filesystem库需要system库支持（`-lboost_system -lboost_filesystem`），位于名字空间boost::filesystem，需要包含头文件`<boost/filesystem.hpp>`

## 常用函数

boost::filesystem是Boost C++ Libraries中的一个模块，主要作用是处理文件（Files）和目录(Directories)。该模块提供的类boost::filesystem::path专门用来处理路径。而且，该模块中还有很多独立的函数能够用来执行创建目录、检查文件是否存在等任务。
二、文件和目录

        该部分包括下列函数：
         boost::filesystem::status(path)                                       查询文件或目录的状态，返回的是boost::filesystem::file_status类型的对象
    
        boost::filesystem::is_directory()                                    根据获取的状态判断是否是目录，返回bool
    
        boost::filesystem::is_empty()                                        判断是否为空
    
        boost::filesystem::is_regular_file()                                 根据获取的状态判断是否是普通文件，返回bool
    
        boost::filesystem::is_symlink()                                      判断符号连接（在windows系统中，后缀为lnk的文件为连接文件）
    
        boost::filesystem::exists()                                              判断是否存在
         boost::filesystem::file_size()                                          返回文件的size，按bytes计算
    
        boost::filesystem::last_write_time()                               返回文件最后一次修改的时间
    
        boost::filesystem::space()                                              返回磁盘的总空间和剩余空间，
    
        boost::filesystem::create_directory()                             创建目录
    
        boost::filesystem::create_directories()                           递归创建整个目录结构
    
        boost::filesystem::remove()                                           删除目录
    
        boost::filesystem::remove_all()                                     递归删除整个目录结构
    
        boost::filesystem::rename()                                           重命名目录
    
        boost::filesystem::copy_file()                                         复制文件
    
        boost::filesystem::copy_directory()                               复制目录
        boost::filesystem::absolute()                                           获取文件或目录的绝对路径
    
        boost::filesystem::current_path()                                   如果没有参数传入，则返回当前工作目录；否则，则将传入的目录设为当前工作目录

# posix_time 模块获取当前的本地时间

```c++
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>

int main() {
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    std::cout << "Current time: " << now << std::endl;
    return 0;
}
```

`boost::posix_time::microsec_clock::local_time()` 这个函数返回一个 `boost::posix_time::ptime` 对象，该对象表示从格林尼治标准时间（GMT）1970年1月1日开始的时间点。

在你的代码中，`rT1` 被赋值为当前的本地时间。这个时间值包含微秒级的精度，因此它可以用于需要高精度时间戳的场景，例如性能分析或者事件日志记录。
