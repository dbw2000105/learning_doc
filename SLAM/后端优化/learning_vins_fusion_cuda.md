# VINS-FUSION-CUDA-BA学习记录

VINS-FUSION-CUDA-BA只对**滑窗中的BA**进行了CUDA加速，而全局BA没有用CUDA加速（参考GlobalOptimization类中仍然在构造ceres的残差块）。

- 统计进行了内存-显存的交换的变量（对应在代码中调用了**cudaMemcpy()**的变量）

```C++
cudaMemcpy()
功能：用于在主机（Host）和设备（Device）之间或设备内部进行内存拷贝操作。
函数原型：
 cudaError_t cudaMemcpy(void* dst, const void* src, size_t count, cudaMemcpyKind kind);
参数说明：
1.dst：目标内存地址(拷贝的目标)。
2.src：源内存地址(拷贝来源)。
3.count：拷贝的字节数。
4.kind：制定拷贝的方向，类型为cudaMemcpyKind，常见值：
    cudaMemcpyHostToDevice：从主机内存拷贝到设备内存。
    cudaMemcpyDeviceToHost：从设备内存拷贝到主机内存。
    cudaMemcpyDeviceToDevice：在设备内存之间拷贝。
    cudaMemcpyHostToHost：在主机内存之间拷贝。
5.返回值：返回cudaSuccess表示拷贝成功，否则返回错误码。
```

| 变量名                                                       | 变量维度                                  | 含义                    |
| ------------------------------------------------------------ | ----------------------------------------- | ----------------------- |
| dev_ptr_states                                               | num_key_frames*16                         | 系统状态量              |
| dev_ptr_ex_para_0                                            | 7                                         | cam0 外参               |
| dev_ptr_ex_para_1                                            | 7                                         | cam1 外参               |
| dev_ptr_cur_td                                               | 1                                         | 当前时延                |
| dev_ptr_states_init                                          | num_key_frames * 16                       | 系统初始状态量          |
| dev_ptr_ex_para_0_init                                       | 7                                         | cam0 初始外参           |
| dev_ptr_ex_para_1_init                                       | 7                                         | cam1 初始外参           |
| dev_ptr_cur_td_init                                          | 1                                         | 初始时延                |
| dev_ptr_inv_depth                                            | num_world_points                          | 特征点逆深度            |
| dev_ptr_inv_depth_init                                       | num_world_points                          | 特征点逆深度初始值      |
| dev_ptr_Hprior                                               | shapes.num_rows_Hpp * shapes.num_cols_Hpp | 先验信息的 Hessian 矩阵 |
| dev_ptr_Bprior                                               | shapes.num_elem_Bpp                       | 先验信息的右端项向量    |
| dev_ptr_Eprior                                               | shapes.num_elem_Bpp                       | 表示先验信息的残差向量  |
| dev_ptr_Jprior                                               | hapes.num_rows_Hpp * shapes.num_cols_Hpp  | 先验信息的雅可比矩阵    |
| **注：上表所有变量的dev_ptr_表示在****GPU****上的指针，其指向显存的一块****内存****的地址，变量维度指的是指向内存的变量的维度。** |                                           |                         |

其中代码中还有一些从GPU到GPU拷贝的操作，拷贝开销较小，先不做考虑。