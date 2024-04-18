# OpenCV中角点提取器

```c++
//FAST
cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
detector->detect(image, keypoints);
//shi-Thomas 
cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create();
detector->detect(image, keypoints);
//ORB
cv::Ptr<cv::ORB> detector = cv::ORB::create();
detector->detect(image, keypoints);
```



#buildOpticalFlowPyramid()函数

 用于构建图像金字塔，以便在光流计算中使用。以下是该函数的用法：

```c++
void cv::buildOpticalFlowPyramid(
    InputArray img, 
    OutputArrayOfArrays pyramid, 
    Size winSize, 
    int maxLevel, 
    bool withDerivatives = true, 
    int pyrBorder = BORDER_REFLECT_101, 
    int derivBorder = BORDER_CONSTANT, 
    bool tryReuseInputImage = true
);
```

其中，参数的含义如下：

- `img`：输入图像，可以是8位或32位浮点数。
- `pyramid`：输出图像金字塔，是一个包含多个层级的向量，每个层级都是一个Mat类型的图像。
- `winSize`：窗口大小，用于计算光流。
- `maxLevel`：金字塔的最大层数。
- `withDerivatives`：是否计算图像的一阶导数。
- `pyrBorder`：金字塔边界的处理方式。
- `derivBorder`：导数边界的处理方式。
- `tryReuseInputImage`：是否尝试重用输入图像。

例如，以下代码将构建一个图像金字塔：

```c++
Mat img = imread("image.jpg");
vector<Mat> pyramid;
Size winSize(15, 15);
int maxLevel = 3;
buildOpticalFlowPyramid(img, pyramid, winSize, maxLevel);
```

# cv::convertPointsToHomogeneous()

用于将二维或三维点集转换为齐次坐标表示。

```c++
void cv::convertPointsToHomogeneous(
    InputArray src, 
    OutputArray dst
);
```

其中，参数的含义如下：

- `src`：输入点集，可以是一个 `Mat` 类型的二维或三维点集。
- `dst`：输出点集，是一个 `Mat` 类型的齐次坐标点集。

例如，以下代码将一个二维点集转换为齐次坐标表示：

```c++
vector<Point2f> points = {Point2f(1, 2), Point2f(3, 4), Point2f(5, 6)};
Mat src(points);
Mat dst;
convertPointsToHomogeneous(src, dst);
```

这将把 `points` 中的点转换为齐次坐标表示，并存储在 `dst` 中。如果输入点集是三维的，则输出点集也是三维的齐次坐标表示。

# cv::projectPoints()

用于将三维点投影到二维图像平面上。

```c++
void cv::projectPoints(
    InputArray objectPoints, 
    InputArray rvec, 
    InputArray tvec, 
    InputArray cameraMatrix, 
    InputArray distCoeffs, 
    OutputArray imagePoints, 
    OutputArray jacobian = noArray(), 
    double aspectRatio = 0
);
```

其中，参数的含义如下：

- `objectPoints`：输入的三维点集，是一个 `Mat` 类型的矩阵，每行表示一个三维点。
- `rvec`：旋转向量，是一个 `Mat` 类型的向量，表示相机的旋转。
- `tvec`：平移向量，是一个 `Mat` 类型的向量，表示相机的平移。
- `cameraMatrix`：相机内参矩阵，是一个 `Mat` 类型的矩阵，包含相机的焦距、主点和畸变参数等信息。
- `distCoeffs`：畸变系数，是一个 `Mat` 类型的向量，包含相机的径向和切向畸变系数。
- `imagePoints`：输出的二维点集，是一个 `Mat` 类型的矩阵，每行表示一个二维点。
- `jacobian`：输出的雅可比矩阵，是一个 `Mat` 类型的矩阵，用于计算相机姿态的导数。
- `aspectRatio`：图像宽高比，用于调整相机内参矩阵的主点位置。

```c++
vector<Point3f> objectPoints = {Point3f(1, 2, 3), Point3f(4, 5, 6), Point3f(7, 8, 9)};
Mat rvec, tvec, cameraMatrix, distCoeffs;
Mat imagePoints;
projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
```

# cv::calcOpticalFlowPyrLK()

用于计算稀疏光流。以下是该函数的用法：

```c++
void cv::calcOpticalFlowPyrLK(
    InputArray prevImg, 
    InputArray nextImg, 
    InputArray prevPts, 
    OutputArray nextPts, 
    OutputArray status, 
    OutputArray err, 
    Size winSize = Size(21, 21), 
    int maxLevel = 3, 
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01), 
    int flags = 0, 
    double minEigThreshold = 1e-4
);
```

其中，参数的含义如下：

- `prevImg`：前一帧图像，可以是一个灰度图像或彩色图像。
- `nextImg`：后一帧图像，与前一帧图像大小和类型相同。
- `prevPts`：前一帧图像中的点集，是一个 `Mat` 类型的二维点集。
- `nextPts`：后一帧图像中的点集，是一个 `Mat` 类型的二维点集，与 `prevPts` 大小和类型相同。
- `status`：输出的状态向量，是一个 `Mat` 类型的向量，每个元素表示对应点的跟踪状态。
- `err`：输出的误差向量，是一个 `Mat` 类型的向量，每个元素表示对应点的跟踪误差。
- `winSize`：窗口大小，用于计算光流。
- `maxLevel`：金字塔的最大层数。
- `criteria`：迭代终止条件，包括最大迭代次数和最小误差阈值。
- `flags`：计算光流的方式，可以是 `0` 或 `OPTFLOW_USE_INITIAL_FLOW`。
- `minEigThreshold`：最小特征值阈值，用于判断是否跟踪成功。

```c++
Mat prevImg = imread("prev.jpg", IMREAD_GRAYSCALE);
Mat nextImg = imread("next.jpg", IMREAD_GRAYSCALE);
vector<Point2f> prevPts = {Point2f(1, 2), Point2f(3, 4), Point2f(5, 6)};
vector<Point2f> nextPts;
Mat status, err;
Size winSize(15, 15);
int maxLevel = 3;
TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
int flags = 0;
double minEigThreshold = 1e-4;
calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err, winSize, maxLevel, criteria, flags, minEigThreshold);
```

这将计算 `prevImg` 和 `nextImg` 之间的稀疏光流，并将结果存储在 `nextPts`、`status` 和 `err` 中。其中，`prevPts` 是前一帧图像中的点集，`nextPts` 是后一帧图像中的点集，`status` 表示每个点的跟踪状态，`err` 表示每个点的跟踪误差。

对flags中OPTFLOW_USE_INITIAL_FLOW的说明：

如果将 `cv::OPTFLOW_USE_INITIAL_FLOW` 作为 `flags` 参数的值，则表示使用初始光流来计算稀疏光流。在实际应用中，**初始光流可以是前一帧图像中的稀疏光流，也可以是其他方法计算得到的光流。**例如，以下代码将使用初始光流来计算稀疏光流：

```c++
Mat prevImg = imread("prev.jpg", IMREAD_GRAYSCALE);
Mat nextImg = imread("next.jpg", IMREAD_GRAYSCALE);
vector<Point2f> prevPts = {Point2f(1, 2), Point2f(3, 4), Point2f(5, 6)};
vector<Point2f> nextPts;
Mat status, err;
Size winSize(15, 15);
int maxLevel = 3;
TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
int flags = OPTFLOW_USE_INITIAL_FLOW;
double minEigThreshold = 1e-4;
calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err, winSize, maxLevel, criteria, flags, minEigThreshold);
```

这将使用初始光流来计算 `prevImg` 和 `nextImg` 之间的稀疏光流，并将结果存储在 `nextPts`、`status` 和 `err` 中。

# noArray()

用于创建一个空的 `cv::Mat` 类型的矩阵。它的定义如下：

```c++
inline Mat noArray() { return Mat(); }
```

它返回一个空的 `Mat` 类型的矩阵，即行数、列数和数据类型都为零的矩阵。在 OpenCV 中，有些函数的参数可以是一个 `Mat` 类型的矩阵，也可以是一个空矩阵，此时可以使用 `noArray()` 来表示一个空矩阵。例如，以下代码将使用 `noArray()` 来表示一个空的输出矩阵:

```c++
Mat src = imread("image.jpg");
Mat dst = noArray();
Canny(src, dst, 50, 100);
```

这将对 `src` 图像进行边缘检测，并将结果存储在 `dst` 中。由于 `dst` 是一个空矩阵，因此 `Canny()` 函数**会自动创建一个与 `src` 大小和类型相同的矩阵来存储结果。**

# TermCriteria()

用于创建一个迭代终止条件。它的定义如下：

```c++
TermCriteria::TermCriteria(
    int type, 
    int maxCount, 
    double epsilon
);
```

其中，参数的含义如下：

- `type`：迭代终止条件的类型，可以是 `TermCriteria::COUNT`、`TermCriteria::EPS` 或 `TermCriteria::COUNT + TermCriteria::EPS`。
- `maxCount`：最大迭代次数，当迭代次数达到该值时，迭代终止。
- `epsilon`：最小误差阈值，当误差小于该值时，迭代终止。

例如，以下代码创建一个最大迭代次数为 100，最小误差阈值为 0.01 的迭代终止条件：

```c++
TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 0.01);
```

这将创建一个 `TermCriteria` 类型的对象 `criteria`，表示当迭代次数达到 100 或误差小于 0.01 时，迭代终止。在实际应用中，`TermCriteria` 类型的对象通常用于计算光流、相机姿态等需要迭代求解的问题。

其中：

+ `TermCriteria::COUNT` 表示最大迭代次数，当迭代次数达到该值时，迭代终止。
+ `TermCriteria::EPS` 表示最小误差阈值，当误差小于该值时，迭代终止。

# Rodrigues()

用于将旋转向量转换为旋转矩阵，或将旋转矩阵转换为旋转向量。它的定义如下：

```c++
void cv::Rodrigues(
    InputArray src, 
    OutputArray dst, 
    OutputArray jacobian = noArray()
);
```

其中，参数的含义如下：

- `src`：输入的旋转向量或旋转矩阵，可以是一个 `Mat` 类型的对象或一个 `Mat` 类型的数组。
- `dst`：输出的旋转矩阵或旋转向量，可以是一个 `Mat` 类型的对象或一个 `Mat` 类型的数组。
- `jacobian`：可选的输出的旋转矩阵或旋转向量的导数，可以是一个 `Mat` 类型的对象或一个 `Mat` 类型的数组。如果不需要导数，可以将其设置为 `noArray()`。

在实际应用中，`cv::Rodrigues()` 函数通常用于将旋转向量转换为旋转矩阵，或将旋转矩阵转换为旋转向量。例如，以下代码将一个旋转向量转换为旋转矩阵：

```c++
Mat rvec(3, 1, CV_64FC1, Scalar(0));
rvec.at<double>(0) = 0.1;
rvec.at<double>(1) = 0.2;
rvec.at<double>(2) = 0.3;
Mat R(3, 3, CV_64FC1);
Rodrigues(rvec, R);
```

这将创建一个大小为 3x1 的旋转向量 `rvec`，其中元素值为 (0.1, 0.2, 0.3)，并将其转换为一个大小为 3x3 的旋转矩阵 `R`。

# Range()

用于表示一个整数范围。它通常用于指定一个矩阵的子矩阵或一个向量的子向量。`cv::Range()` 的构造函数有两种形式：

```c++
Range::Range(int _start, int _end);
Range::Range(int _start, int _end, int _step);
```

其中，第一种形式表示一个起始于 `_start`，终止于 `_end-1` 的整数范围，即左闭右开区间 `[start, end)`。第二种形式表示一个起始于 `_start`，终止于 `_end-1`，步长为 `_step` 的整数范围。例如，`Range(0, 3)` 表示整数范围 `[0, 1, 2]`，`Range(0, 6, 2)` 表示整数范围 `[0, 2, 4]`。#

在实际应用中，`cv::Range()` 通常用于指定一个矩阵的子矩阵或一个向量的子向量。例如，以下代码创建一个大小为 3x3 的矩阵 `mat`，并提取其第一行和第二列：

```c++
cv::Mat mat(3, 3, CV_32FC1);
cv::Range row_range(0, 1);
cv::Range col_range(1, 2);
cv::Mat submat = mat(row_range, col_range);
```

这将创建一个大小为 3x3 的单通道浮点型矩阵 `mat`，然后创建两个 `cv::Range` 对象 `row_range` 和 `col_range`，分别表示第 0 行和第 1 列。最后，使用 `row_range` 和 `col_range` 提取 `mat` 的第一行和第二列，得到一个大小为 1x1 的子矩阵 `submat`。
