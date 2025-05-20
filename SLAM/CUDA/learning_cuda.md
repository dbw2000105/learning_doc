# CUDA编程学习

## 常用函数及说明

### 内存操作

#### cudaMemcpy()

功能：用于在主机（Host）和设备（Device）之间或设备内部进行**内存拷贝操作**。

函数原型：

```c++
cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind kind);
```

参数说明：

1. `dst`：目标内存地址(拷贝的目标)。

2. `src`：源内存地址(拷贝来源)。

3. `count`：拷贝的字节数。

4. `kind`：制定拷贝的方向，类型为cudaMemcpyKind，常见值：

- cudaMemcpyHostToDevice：从主机内存拷贝到设备内存。
- cudaMemcpyDeviceToHost：从设备内存拷贝到主机内存。
- cudaMemcpyDeviceToDevice：在设备内存之间拷贝。
- cudaMemcpyHostToHost：在主机内存之间拷贝。

5. `返回值`：返回`cudaSuccess`表示拷贝成功，否则返回错误码。

#### cudaMalloc()

功能：用于在设备（GPU）上分配内存。

函数原型：

```c++
cudaError_t cudaMalloc(void** devPtr, size_t size);
```

参数说明：

1. `devPtr`：

* 指向设备内存指针的指针。
* 分配成功后，`devPtr` 将指向分配的设备内存地址。

2. `size`：要分配的内存大小（以字节为单位）。
3. `返回值`：返回`cudaSuccess`表示分配成功，如果分配失败，会返回错误代码（如内存不足）。

#### cudaMemset()

功能：用于在设备（GPU）内存中将指定的内存区域初始化为某个值。

函数原型：

```c++
cudaError_t cudaMemset(void* devPtr, int value, size_t count);
```

参数说明：

1. `devPtr` ：指向设备内存的指针。

2. `value`：用于初始化内存的值，该值会被解释为一个字节（`unsigned char`），并填充到内存中。

3. `count`：要初始化的字节数
4. `返回值`：返回 `cudaSuccess` 表示操作成功，否则返回错误码

### 矩阵操作

两个常用的库：**cuBLAS** 和**cuSOLVER**

**cuBLAS** 是 NVIDIA 提供的一个高性能 GPU 加速的 **BLAS（Basic Linear Algebra Subprograms）库**，用于在 GPU 上执行线性代数操作。它是 CUDA 的一部分，专门为矩阵和向量运算提供优化的实现。

1. 向量操作
   - 如向量加法、标量乘法、点积等。
2. 矩阵操作
   - 如矩阵加法、矩阵乘法、矩阵转置等。
3. 矩阵分解
   - 如 LU 分解、QR 分解等（部分功能需要配合 cuSOLVER）。
4. 其他高级操作
   - 如求解线性方程组、特征值分解等。

**cuSOLVER** 是 NVIDIA 提供的一个高性能线性代数库，专门用于在 GPU 上执行**矩阵分解**和**求解操作**。它支持以下功能：

1. 矩阵分解
   - LU 分解、QR 分解、Cholesky 分解等。
2. 线性方程组求解
   - 使用矩阵分解方法求解线性方程组。
3. 特征值分解：
   - 计算矩阵的特征值和特征向量。
4. 奇异值分解（SVD）
   - 计算矩阵的奇异值分解。

#### cublasCreate_v2()

功能：是 CUDA 的 cuBLAS 库中的一个函数，用于创建一个 cuBLAS 上下文（handle）。它的作用是初始化 cuBLAS 库，并返回一个句柄（`cublasHandle_t`），供后续的 cuBLAS 函数调用使用。

函数原型：

```c++
cublasStatus_t cublasCreate_v2(cublasHandle_t* handle);
```

参数说明：

1. `handle`：指向 `cublasHandle_t` 类型的指针。创建成功后，`handle` 将指向一个 cuBLAS 上下文，用于后续的 cuBLAS 操作。
2. `返回值`：返回CUBLAS_STATUS_SUCCESS表示创建成功，如果失败，则返回其他错误代码。

#### cublasDestroy_v2()

功能：销毁cuBLAS上下文：

```c++
cublasStatus_t cublasDestroy_v2(cublasHandle_t handle);
```

参数说明与1.1.2.1同理。

#### cusolverDnCreate()

功能：创建一个 cuSOLVER Dense 上下文（handle）。它的作用是初始化 cuSOLVER Dense 库，并返回一个句柄（`cusolverDnHandle_t`），供后续的 cuSOLVER Dense 函数调用使用。

函数原型：

```c++
cusolverStatus_t cusolverDnCreate(cusolverDnHandle_t* handle);
```

参数说明与1.1.2.1同理。

### 流操作

#### cudaStreamSynchronize()

功能：用于同步指定的 CUDA 流（`cudaStream_t`）。它的作用是阻塞主机线程，直到指定流中的所有任务（如内核函数、内存拷贝等）都完成。

函数原型：

```c++
cudaError_t cudaStreamSynchronize(cudaStream_t stream);
```

参数说明：

1. `stream`：要同步的CUDA流，如果传入0，表示同步默认流(cudaStreamDefault)。
2. 返回值：返回 `cudaSuccess` 表示同步成功，否则返回其他错误代码。

经典用法：

```c++
cudaError_t cuda_status = cudaStreamSynchronize(0);
assert(cuda_status == cudaSuccess);
```

## 核函数及启动方法

### 基本概念

在 CUDA 编程中，**核函数（Kernel Function）** 是在 GPU 上并行执行的函数。其启动方式和变量定义与普通 CPU 函数不同。

核函数通过 **三重尖括号 `<<< >>>`** 语法启动，语法形式为：

```cpp
kernel_name<<<grid_config, block_config, shared_mem_size, stream>>>(arguments...);
```

参数含义：

1. **`grid_config`**
   - 定义 **网格（Grid）** 的维度，即线程块（Block）在网格中的布局。
   - 可以是 `dim3` 类型（支持 1D/2D/3D）或直接使用整数值（仅 1D）。
   - 例如：`dim3 grid(2, 3)` 表示网格有 `2x3` 个线程块。
2. **`block_config`**
   - 定义 **线程块（Block）** 的维度，即线程（Thread）在线程块中的布局。
   - 同样可以是 `dim3` 或整数值。
   - 例如：`dim3 block(16, 16)` 表示每个线程块有 `16x16` 个线程。
3. **`shared_mem_size`** (可选)
   - 指定每个线程块使用的 **共享内存大小**（字节数），默认为 0。
   - 例如：`sizeof(float) * 64` 表示每个线程块分配 64 个 float 的共享内存。
4. **`stream`** (可选)
   - 指定核函数运行的 **CUDA 流**（Stream），默认为 0（默认流）。

### 核函数的内置变量

在核函数内部，CUDA 提供了以下 **内置变量** 用于确定线程的全局/局部位置：

|   变量名    | 类型   | 含义                                                       |
| :---------: | ------ | ---------------------------------------------------------- |
| `threadIdx` | `dim3` | 当前线程在 **线程块内的索引**（范围：`[0, blockDim-1]`）。 |
| `blockIdx`  | `dim3` | 当前线程块在 **网格内的索引**（范围：`[0, gridDim-1]`）。  |
| `blockDim`  | `dim3` | 线程块的维度（即每个线程块包含的线程数）。                 |
|  `gridDim`  | `dim3` | 网格的维度（即网格包含的线程块数）。                       |
| `warpSize`  | `int`  | Warp 的大小（通常为 32）。                                 |
