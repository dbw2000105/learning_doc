[TOC]

# Eigen子矩阵操作(block)

|              块操作               |  构建动态大小子矩阵   |
| :-------------------------------: | :-------------------: |
| 提取块大小为`(p,q)`,起始于`(i,j)` | matrix.block(i,j,p,q) |
|         m.block<p,q>(i,j)         |                       |

注意索引是从0开始，例：

```c++
//m.block<p,q>(i,j)
//m.block(i,j,p,q)
MatrixXf m(4,4);
    m<< 1,2,3,4,
        5,6,7,8,
        9,10,11,12,
        13,14,15,16;
//方法一
cout<<m.block<2,2>(1,1)<<endl<<endl;
//方法二
cout<<m.block(0,0,2,2)<<endl<<endl;

//方法一输出：
 6  7
10 11
//方法二输出：
1 2
5 6     
```

# 向量的块操作：

```c++
获取向量的前n个元素：[vector].head(n);

获取向量尾部的n个元素：vector.tail(n);

获取从向量的第i个元素开始的n个元素：vector.segment(i,n);
```

# 对矩阵的行列操作

## leftCols()和rightCols()

`leftCols()` 函数用于获取矩阵的左侧若干列，其参数为列数。例如，`A.leftCols(n)` 表示获取矩阵 `A` 的前 `n` 列，返回一个 `Eigen::Matrix` 类型的矩阵。

`rightCols()` 函数用于获取矩阵的右侧若干列，其参数为列数。例如，`A.rightCols(n)` 表示获取矩阵 `A` 的后 `n` 列，返回一个 `Eigen::Matrix` 类型的矩阵。

+ 需要注意的是，`leftCols()` 和 `rightCols()` 函数返回的矩阵**与原矩阵共享内存**，即它们是原矩阵的一个视图，而不是拷贝。因此，对返回的矩阵进行修改会影响原矩阵。

+ 需要注意的是，leftCols和rightCols的系数都是从1开始的

`leftCols<n>()` 是一个模板函数，其中的模板参数 n表示要获取的列数。**这种写法可以在编译时确定要获取的列数，因此具有更好的类型安全性和效率**。但是，如果要获取的列数是一个变量，就不能使用 `leftCols<>()`，而必须使用 `leftCols()`。

```c++
#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 5);
    std::cout << "A:\n" << A << std::endl;

    Eigen::MatrixXd B = A.leftCols<2>();
    std::cout << "B:\n" << B << std::endl;

    Eigen::MatrixXd C = A.leftCols(2);
    std::cout << "C:\n" << C << std::endl;

    return 0;
}
```

## topRows()和bottomRows()

- `topRows()`：获取矩阵的前若干行，其参数为行数。例如，`A.topRows(n)` 表示获取矩阵 `A` 的前 `n` 行，返回一个 `Eigen::Matrix` 类型的矩阵。
- `bottomRows()`：获取矩阵的后若干行，其参数为行数。例如，`A.bottomRows(n)` 表示获取矩阵 `A` 的后 `n` 行，返回一个 `Eigen::Matrix` 类型的矩阵。

## row()和col()

- `row()`：获取矩阵的某一行，其参数为行索引。例如，`A.row(i)` 表示获取矩阵 `A` 的第 `i` 行，返回一个 `Eigen::Matrix` 类型的行向量。
- `col()`：获取矩阵的某一列，其参数为列索引。例如，`A.col(j)` 表示获取矩阵 `A` 的第 `j` 列，返回一个 `Eigen::Matrix` 类型的列向量。

# Eigen 中对矩阵进行奇异值分解

直接使用Eigen::JacobiSVD即可

```c++
#include <Eigen/Dense>
#include <iostream>

int main() {
    // 创建一个示例矩阵
    Eigen::MatrixXd matrix(3, 3);
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;

    std::cout << "Original matrix:\n" << matrix << std::endl;

    // 使用JacobiSVD计算奇异值
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // 获取奇异值
    Eigen::VectorXd singularValues = svd.singularValues();

    std::cout << "Singular values:\n" << singularValues << std::endl;
	// 获取U和V矩阵
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    return 0;
}
```

需要注意：Eigen中的SVD分解直接可以得到U，$\sigma$，V，而不是V的转置。

SVD分解的原式子如下：
$$
A=U\Sigma V^T
$$
这里面V是有转置的。

而如果调用python中的svd分解函数(numpy或者scipy)，返回值得到的是U，$\sigma$，$V^T$

**在SVD求解三角化的时候，就很容易对矩阵搞错**

注意三角化的最后一步需要对矩阵D进行SVD分解，有两种方法可以求得：对$D^TD$进行分解，对$D$分解

+ 如果对$D^TD$分解，可知任意矩阵的转置乘他本身一定是一个对称矩阵，对对称矩阵进行SVD分解就变成了cholesky分解：

$$
A = U\Sigma U^T
$$

这时候直接拿U矩阵进行计算即可。

+ 如果对$D$分解，得到的就是常规SVD分解的值，这时候需要拿V矩阵(注意python中需要对返回值求转置，因为拿到的是$V^T$)



# LDLT分解

`.ldlt()` 是 Eigen 中的一个函数，用于对一个对称正定矩阵进行 LDLT 分解，即将矩阵分解为一个下三角矩阵 L、一个对角线矩阵 D 和它的转置矩阵 L^T 的乘积，即 A = L * D * L^T。这种分解可以有效地解决线性方程组的求解问题。

`.solve()` 是 Eigen 中的一个函数，用于求解线性方程组 Ax = b，其中 A 是一个矩阵，b 是一个向量，x 是一个未知向量。该函数的参数可以是一个向量，也可以是一个矩阵，分别表示求解单个方程和多个方程的解。

一般会两者一起使用。

```c++
//Ax=b
//使用方法：
auto x = A.ldlt().solve(b); 
```

# QR分解

## 稠密矩阵的QR分解

```c++
#include <iostream>
#include <Eigen/Dense>

int main()
{
    Eigen::MatrixXd A(3, 3);
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

    Eigen::HouseholderQR<Eigen::MatrixXd> qr_helper(A);
    Eigen::MatrixXd Q = qr_helper.householderQ();
    Eigen::MatrixXd R = qr_helper.matrixQR().triangularView<Eigen::Upper>();

    std::cout << "Q:\n" << Q << std::endl;
    std::cout << "R:\n" << R << std::endl;

    return 0;
}
```

## 稀疏矩阵的QR分解

`SPQR` 是 Eigen 中的一个模板类，用于求解稀疏矩阵的 QR 分解。

`SPQR` 类的模板参数是稀疏矩阵的类型，例如 `SPQR<SparseMatrix<double>>` 表示对一个元素类型为 `double` 的稀疏矩阵进行 QR 分解。

以下是一个示例代码，演示了 `SPQR` 类的用法：

```c++
#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

int main()
{
    Eigen::SparseMatrix<double> A(3, 3);
    A.insert(0, 0) = 1.0;
    A.insert(1, 1) = 2.0;
    A.insert(2, 2) = 3.0;
    A.makeCompressed();

    Eigen::SPQR<Eigen::SparseMatrix<double>> qr(A);

    std::cout << "Q:\n" << qr.matrixQ() << std::endl;
    std::cout << "R:\n" << qr.matrixR() << std::endl;

    return 0;
}
```

对于QR分解后的成员函数：（通过"."调用）

+ setSPQROrdering()：这行代码是在设置 QR 分解的选项中的一个，用于指定 QR 分解中的列排序方式。`SPQR_ORDERING_NATURAL` 表示使用自然排序，即按照列的顺序进行排序。除此之外，还可以使用其他的列排序方式，例如 `SPQR_ORDERING_COLAMD` 表示使用列主元最小度数法进行排序。

- `matrixQ()`：返回正交矩阵 Q。

- `matrixR()`：返回上三角矩阵 R。

- compute()：一般需要调用此函数开始稀疏矩阵的QR分解。

  

# Eigen 中的别名操作（Aliasing）

在Eigen中，别名是指在赋值表达式中，同一个矩阵(数组、向量)同时出现在赋值运算符的左侧和右侧。

例如表达式`mat = 2 * mat; 和 mat = mat.transpose()` 就是别名。第一个别名是合法的，第二个别名会导致非预期结果。

使用.eval()可以解决别名问题。

Eigen把右侧值全部写入一个临时矩阵/数组中，然后将它赋值给左侧。函数eval()就是这么做的。

# squaredNorm()函数

`squaredNorm()`是`Eigen::Vector3f`类的成员函数，它返回向量的**平方欧几里得范数**。如下示例：

```c++
int main(){
	Eigen::Vector3f point<<1,2,3;
    point.squaredNorm() //求此向量的二范数，即模长，也可以表示距离    
}
```

# Isometry3d

`Isometry3d` 是 Eigen 库中的一个类，表示一个三维欧几里得变换矩阵。它是 `Eigen::Transform<double, 3, Eigen::Isometry>` 类的别名，其中 `double` 表示矩阵元素的类型，`3` 表示矩阵的维度，`Eigen::Isometry` 表示变换类型，表示该矩阵是一个等距变换矩阵。

`Isometry3d` 类提供了一些成员函数，用于进行矩阵的初始化、赋值、乘法、逆运算等操作。例如，以下代码创建了一个 `Isometry3d` 对象 `T`，并将其初始化为一个单位矩阵：

```c++
#include <Eigen/Geometry>
Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
```

可以使用 `matrix()` 函数获取 `Isometry3d` 对象的矩阵表示，例如：

```c++
Eigen::Matrix4d mat = T.matrix();
```

可以使用 `inverse()` 函数获取 `Isometry3d` 对象的逆矩阵，例如：

```c++
Eigen::Isometry3d T_inv = T.inverse();
```

可以使用 `translation()` 函数获取 `Isometry3d` 对象的平移向量，例如：

```c++
Eigen::Vector3d t = T.translation();
```

可以使用 `rotation()` 函数获取 `Isometry3d` 对象的旋转矩阵，例如：

```c++
Eigen::Matrix3d R = T.rotation();
```

需要注意的是，`Isometry3d` 类是一个模板类，可以使用不同的数据类型作为矩阵元素的类型。例如，可以使用 `float` 或 `long double` 等类型代替 `double`。

或者使用linear()函数获取对象的旋转矩阵

```c++
#include <Eigen/Geometry>

Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
T.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

Eigen::Matrix3d R = T.linear();
```

+ linear()和rotation()函数的区别：

`Eigen::Isometry3d` 类中的 `linear()` 和 `rotation()` 函数都可以用于获取该变换矩阵的旋转部分，但它们的**返回值类型不同**

`linear()` 函数的返回值是一个 `Eigen::Matrix3d` 类型的矩阵，表示该变换矩阵的旋转部分，即**旋转矩阵**。

`rotation()` 函数的返回值是一个 `Eigen::Quaterniond` 类型的四元数，表示该变换矩阵的旋转部分，即**旋转四元数**。

# conservativeResize()

用于调整矩阵的大小，并保留原有的数据。它可以用于动态调整矩阵的大小，例如在矩阵中添加或删除行或列。

`conservativeResize()` 函数有两个参数，第一个参数是要调整大小的维度，可以是 `Eigen::NoChange`、`Eigen::Dynamic` 或者一个正整数，第二个参数是要保留的数据的起始位置，默认为 `(0, 0)`。

```c++
#include <Eigen/Core>

Eigen::MatrixXd mat(3, 3);
mat << 1, 2, 3,
       4, 5, 6,
       7, 8, 9;

mat.conservativeResize(4, 4);
mat.block(3, 0, 1, 3) << 10, 11, 12;
mat.block(0, 3, 3, 1) << 13, 14, 15;

std::cout << mat << std::endl;
```

在上面的示例中，首先创建了一个大小为 3x3 的矩阵 `mat`，并将其初始化为一个 3x3 的单位矩阵。然后使用 `conservativeResize()` 函数将 `mat` 的大小调整为 4x4，并保留原有的数据。接着，使用 `block()` 函数将新添加的一行和一列的数据分别赋值为 10、11、12 和 13、14、15。最后，输出调整后的矩阵 `mat`。

需要注意的是，`conservativeResize()` 函数只能用于调整矩阵的大小，不能用于改变矩阵的维度。如果要改变矩阵的维度，可以使用 `resize()` 函数。另外，`conservativeResize()` 函数只能用于动态大小的矩阵，不能用于静态大小的矩阵。

#   SparseMatrix 类型

`SparseMatrix` 是 Eigen 中用于表示稀疏矩阵的类。稀疏矩阵是指矩阵中大部分元素都为零的矩阵，与之相对的是密集矩阵，即矩阵中大部分元素都不为零的矩阵。由于稀疏矩阵中大部分元素都为零，因此可以采用特殊的存储方式来节省存储空间和计算时间。

`SparseMatrix` 类的模板参数包括矩阵元素的类型、行索引的类型和列索引的类型。例如，`SparseMatrix<double, Eigen::RowMajor>` 表示一个元素类型为 `double` 的稀疏矩阵，行索引为 `int` 类型，列索引为 `int` 类型，行优先存储。

以下是一个示例代码，演示了 `SparseMatrix` 类的用法：

```c++
#include <iostream>
#include <Eigen/Sparse>

int main()
{
    Eigen::SparseMatrix<double> A(3, 3);
    A.insert(0, 0) = 1.0;
    A.insert(1, 1) = 2.0;
    A.insert(2, 2) = 3.0;
    A.makeCompressed();

    std::cout << "A:\n" << A << std::endl;

    return 0;
}
```

输出结果如下：

```
A:
  (0, 0)        1
  (1, 1)        2
  (2, 2)        3
```

可以看到，该代码创建了一个 $3\times 3$ 的稀疏矩阵 `A`，并将其对角线元素设置为 $1,2,3$。`A.insert(i, j) = x` 表示将矩阵 `A` 的第 `i` 行、第 `j` 列的元素设置为 `x`。最后，`A.makeCompressed()` 表示将稀疏矩阵压缩存储，以便于后续的计算。

## 关于稀疏矩阵的一些操作

```cpp
Eigen::MatrixXd Q(3, 3);
Q.selfadjointView<Eigen::Upper>();
```

它创建并返回一个对称矩阵的上三角自共轭视图。在这个特定的例子中，使用 `Eigen::Upper` 意味着你只关心上三角部分的元素，而下三角部分将自动被相应的上三角元素填充，以使整个矩阵保持对称。

## AngleAxisd

`AngleAxisd` 是 Eigen 中的一个类，用于表示绕任意轴旋转的角度和旋转轴。

`AngleAxisd` 类的成员函数包括：

- `angle()`：返回旋转角度。
- `axis()`：返回旋转轴。
- `fromRotationMatrix(R)`：从旋转矩阵构造 `AngleAxisd` 对象。
- `toRotationMatrix()`：将 `AngleAxisd` 对象转换为旋转矩阵。
- `inverse()`：返回逆旋转。
- `operator*=(const AngleAxisd& other)`：将当前旋转与另一个旋转相乘。
- `operator*(const AngleAxisd& other) const`：返回当前旋转与另一个旋转相乘的结果。

需要注意的是，`AngleAxisd` 类中的旋转角度单位为弧度。如果需要使用角度作为单位，可以使用 `Eigen::AngleAxisd::fromAngleAxis()` 和 `Eigen::AngleAxisd::toAngleAxis()` 函数进行转换。

```c++
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main()
{
    Eigen::Matrix3d R;
    R << 0.0, 0.0, 1.0,
         1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;

    Eigen::AngleAxisd aa(R);
    std::cout << "Angle: " << aa.angle() << std::endl;
    std::cout << "Axis: " << aa.axis().transpose() << std::endl;

    Eigen::Matrix3d R2 = aa.toRotationMatrix();
    std::cout << "R2:\n" << R2 << std::endl;

    return 0;
}
```

