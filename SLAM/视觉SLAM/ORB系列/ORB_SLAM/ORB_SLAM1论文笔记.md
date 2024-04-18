#摘要

## 单词积累

1. clutter n.杂乱的东西 v. 乱堆,塞满   

2. from scratch 从头开始

3. survival of the fittest 适者生存

4. compact adj.小型的,紧凑的    v.压紧,压实

   区分:compatible adj.兼容的,可共存的
5. exhaustive adj.详尽的,彻底的,消耗的
6. with respect to 关于
7. state-of-the-art 最先进的;已经发展的;达到最高水准的
8. prohibitive adj.(费用)高得令人望而却步的;限制性的,禁止的

## 重点句子理解分析

* uses the same features for all SLAM tasks: tracking, mapping, relocalization, and loop closing.

ORB_SLAM的特点:对所有的SLAM任务使用相同的特征:追踪,重定位以及回环检测.

* A survival of the fittest strategy that ==selects the points and keyframes of the reconstruction== leads to excellent robustness and generates a compact and trackable map that only grows if the scene content changes, allowing lifelong operation.

选择重建的点和关键帧这种策略可以获得很好的鲁棒性,并且可以创建一个小型的子图,这种子图只有在场景变化的时候才会增长.

# 第一部分 介绍

## 单词积累 语法积累

1. sparse adj 稀疏的,稀少的

2. given that 做条件状语

Bundle adjustment (BA) is known to provide accurate estimates of camera localizations as well as a sparse geometrical reconstruction , ==given that== a strong network of matches and good initial guesses are provided.

**分析**:前面半句主谓宾没什么问题,这里given that 后引导一个**条件状语从句**,表示在这个条件下,主句才成立.

翻译:众所周知,BA能够提供精确的相机定位与稀疏地图重建,前提是需要提供一个强大的匹配网络与很好的初始估计.

一些其他的条件状语引导词:if、unless、as/so long as、once、in case、on condition that、supposing (that)、providing (that)、provided (that)

3. prohibitive 

## 重点句子理解分析

==一个实时的SLAM系统如果想要使用BA必须满足以下六点:==

* Corresponding observations of scene features (map points) among a subset of selected frames (keyframes).

**1.地图点要在所选关键帧的子集中**

* As complexity grows with the number of keyframes, their selection should avoid unnecessary redundancy.

**2.当复杂度随着关键帧数量的增加而增加时，它们的选择应该避免不必要的冗余。**

* A strong network configuration of keyframes and points to produce accurate results, that is, a well spread set of keyframes observing points with significant parallax and with plenty of loop closure matches.*

**3.一个强大的由关键帧和地图点组成的网络配置能够产生精确的结果,也就是说,一组分散的能够观察到地图点的关键帧具有显著的视差以及很多回环约束的匹配**

* An initial estimation of the keyframe poses and point locations for the non-linear optimization.

**4.对于非线性优化需要一个关键帧位姿和地图点位置的初始的估计**

* A local map in exploration where optimization is focused to achieve scalability.

**5.一个在系统运行过程中的局部地图,其中优化的重点是实现可伸缩性**

* The ability to perform fast global optimizations (e.g. pose graph) to close loops in real-time.

**6.能够执行快速全局优化(例如姿态图)实时关闭循环。**

==PTAM的优缺点:==

优点:该算法在规模较小的情况下，为关键帧选择、特征匹配、三角化、每帧相机定位以及跟踪失败后的重新定位提供了简单而有效的方法。

缺点:缺乏回环检测和足够的遮挡处理，对重定位的视角不变性低，地图引导需要人为干预 

==ORB_SLAM1的主要贡献:==

1.对所有的任务使用相同的特征(追踪,建图,重定位以及回环),使用ORB特征,这是一种允许实时运行的,不需要gpu,且对视角和光照的不变性较好.

2.在大场景下能实时运行,由于使用**共视图**,追踪和建图关注**局部的共视区域**,不受全局地图大小的影响

3.基于位姿图优化的实时回环检测，我们称之为**本质图**。它由系统维护的生成树、回环链接和共视图中的强边组成。

4.实时摄像机重定位，对视点和照明具有显著的不变性。这允许从跟踪失败中恢复，还增强了地图的复用

5.基于模型选择的一种新的自动和鲁棒的初始化过程，允许创建平面和非平面场景的初始地图。

6.一种适者生存的方法来选择地图点和关键帧，在生成关键帧方面是慷慨的，但在剔除方面是限制性的。由于丢弃了冗余关键帧，该策略提高了跟踪的鲁棒性，并加强了长期运行的能力。

# 第二部分 相关工作

## 单词积累

one order of magnitude   一个数量级

high recall 高召回率

heuristic adj 启发性的,探索性的 n 启发式步骤（或方法）

takes into account 考虑

rationale n 基本原理,全部理由,根本原因

for the sake of 为了...的目的

refrain v 克制

drawback n 缺点
