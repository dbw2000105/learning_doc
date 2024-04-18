

# yaml文件使用

cmake中需要添加: 

使用yaml-cpp: target_link_libraries(test /usr/local/lib/libyaml-cpp.a)

使用opencv的FileStorage: find_package(OpenCV REQUIRED)   target_link_libraries(test ${OpenCV_LIBS})

## 基本规则

YAML有以下基本规则：
 1、大小写敏感
 2、使用缩进表示层级关系
 3、禁止使用tab缩进，只能使用空格键
 4、缩进长度没有限制，只要元素对齐就表示这些元素属于一个层级。
 5、使用#表示注释
 6、字符串可以不用引号标注

## 简单实例



yaml文件:

```yaml
name: frank
sex: male
age: 18

skills: 
  c++: 1
  java: 1
  android: 1
  python: 1
```

```c++
#include <iostream>
#include "yaml-cpp/yaml.h"

using namespace std;

int main(int argc,char** argv)
{
    YAML::Node config = YAML::LoadFile("../config.yaml");

    cout << "name:" << config["name"].as<string>() << endl;
    cout << "sex:" << config["sex"].as<string>() << endl;
    cout << "age:" << config["age"].as<int>() << endl;
    return 0;
}
```



# 从yaml文件中读取矩阵的方法:

## 使用vector容器读取矩阵

核心思想是先使用vector将数据保存下来之后再赋值给eigen矩阵

```c++
#include <iostream>
#include "yaml-cpp/yaml.h" //导入yaml-cpp文件
#include <Eigen/Core>
#include <vector>
using namespace std;
int main(){
    YAML::Node config = YAML::LoadFile("../config.yaml");
    Eigen::Matrix3d k;
    //先创建一个vector容器将yaml文件的内容存起来
    vector<int> a = config["k_matrix"]["data"].as<vector<int>>();
    //这一步是为了固定vector的长度,可以不写
    assert((int)a.size() == 9);
    //循环遍历将数据存入eigen矩阵中
    int t = 0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            k(i,j) = a[t];
            t++;
        }
    }
    cout<<k;
}
```

yaml文件:

```yaml
k_matrix: 
  rows: 3
  cols: 3
  dt: d
  data: [1,0,0,0,1,0,0,0,1]
```

## 使用opencv读取矩阵

核心思想是通过cv的方法cv::FileStorage读取yaml文件,再通过函数cv2eigen()将cv矩阵转化为eigen矩阵,需要注意的是需要引入头文件<opencv2/core/eigen.hpp>

```c++
#include <iostream>
#include <Eigen/Core>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
int main(){

    cv::FileStorage file_settings("../config.yaml", cv::FileStorage::READ);
    Eigen::Matrix3d k;
    Mat b;
    file_settings["k_matrix"]>> b;
    cv2eigen(b,k);
    cout<<k<<endl;
}
```

yaml文件:

```yaml
%YAML:1.0 #注意这行无空格
---
k_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [1,0,0,0,1,0,0,0,1]
```

需要注意的是使用opencv的方式读取yaml文件的时候需要引入格式**%YAML:1.0 ,--- **以及需要在矩阵后添加

**!!opencv-matrix**

