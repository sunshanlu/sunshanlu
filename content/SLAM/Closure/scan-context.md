---
authors: 孙善路-github, 孙善路-bilibili
title: `Scan Context` 和 `Scan Context++`算法说明
tags: SLAM
date: 2025-03-01
slug: scan-context
Category: SLAM, 回环检测
description: 本篇文章会对`Scan Context` 和 `Scan Context++`算法细节和流程进行说明，为后续在FAST-LIVO2中的使用打下基础
---

[TOC]


## 1. `Scan Context`

`Scan Context`论文通过**构建旋转不变**的`PC`描述子实现基于点云结构的回环闭合检测功能。

### 1.1 `PC`描述子构建过程

`PC`描述子的构建在极坐标下，将点云信息编码成`2.5D`的描述子信息。以雷达坐标系为中心，在`BEV`视角下沿着极坐标系的轴向和角度方向划分`bin`，然后`bin`中保存着**最高**点对应的**z坐标值**，这个过程可以认定为点云将采样过程，其中轴向和角度方向可以由$N_r$和$N_s$两个超参数表示，同时也控制着得到`PC`描述子的分辨率，`PC`描述子的构建过程如下图所示，其中蓝色的点为空`bin`对应的颜色，`PC`描述子矩阵将其编码为`0`。`PC`描述子矩阵中，`row`编码着下图(a)黄色圈`bin`，`col`编码着下图(a)青色`bin`。使用最高点的高度作为编码的原因是作者认为，最高点可以较大程度的描述一个区域内的特征。

想象一下，当激光雷达的航向角发生变化时，`PC`描述子矩阵的`col`可能会发生整体左移，且超出列边界的列会依次填补到最左侧。因此当激光雷达仅发生航向角变化时，我们总可以找到一个列左移数$n$，使得两个点云对应的`PC`描述子矩阵完全相同，这也恰恰证明了`PC`描述子的旋转不变性。根据这个逻辑，作者定义了由`PC`描述子表示的两个点云之间的相关性公式如下。

$$
\begin{align}
    d(I^{q},I^c)&=1-\frac{1}{N_s}\sum_{j=1}^{N_s}{\frac{c_j^q\cdot c_j^c}{||c_j^q||||c_j^c||}}\\
    D(I^q,I^c)&=\min_{n\in [N_s]}{d(I^q,I_n^c)}\\
    n^*&=\arg \min_{n\in [N_s]} {d(I^q,I_n^c)}
\end{align}
$$

式中：
- $I^{q}$，当前激光雷达点云对应的`PC`描述子矩阵；
- $I^c$，为数据库中雷达点云对应的`PC`描述子矩阵；
- $I^c_n$，为数据库中雷达点云对应的`PC`描述子矩阵列左移动`n`列后得到的`PC`描述子矩阵；
- $d(I^{q},I^c)$，为1-矩阵平均列余弦相似度，作为矩阵距离；
- $D(I^q,I^c)$，为当前激光雷达点云描述子$I^{q}$与数据库中某个点云描述子$I^c$的距离；
- $n^*$为最小距离对应行左移索引，此外，这个左移索引对应的旋转角度还可以**作为$R_{cq}$旋转矩阵的初值**。


<div align='center'>
    <image src="../images/scan_context.png" alt="描述子构建过程" width=800px/>
</div>

!!! note "左移索引与旋转初值的关系"
    当数据库中的描述子矩阵和当前描述子矩阵为同一地点的不同角时，假设数据库对应描述子沿z轴逆时针旋转`k`个角度单位时，对应描述子矩阵列必然向左移动`n`个单位，这是因为世界坐标系下的静态物体在移动过程中，世界坐标不会发生改变，但在雷达坐标系下却向右旋转了而，反应在描述子矩阵上为向左移动的一段距离。这时`n`乘单位`bin`对应的角度即可描述旋转矩阵$R_{cq}$的初值。

### 1.2 `Scan Context`算法思路

从`PC`描述子的相似度计算公式上来看，要计算当前点云和数据库中点云的相似程度，需要对数据库中的所有`PC`描述子进行暴力检索，为了提高计算效率，`Scan Context`算法提出了`Ring key`描述符来加快回环闭合点云检索过程。`Ring Key`的计算公式可以由下面的公式描述：

$$
\begin{align}
    &k_r&=&[\varphi(r_1),..., \varphi(r_{N_r})]^T\\
    &\varphi(r_i)&=&\frac{||r_i||_0}{N_s}
\end{align}
$$

其中，

- $r_i$代表`PC`描述子矩阵第$i$行，代表一圈描述子信息；
- $\varphi(r_i)$为`PC`描述符的行编码公式，由平均`L0`范数描述，即计算行中非零元素的个数，并除以`N_s`，得到行编码；
- $k_r$为`Ring Key`，其为所有行得到的计算结果的集合。

通过上面描述的`Ring Key`计算公式，可以由`PC`描述子计算得到行编码值组成的`Ring Key`向量，由行方向上非`0`块数量近似代表这部分的点云特征。`Scan Context`使用`kdtree`存储`Ring Key`向量，并使用`kdtree`进行检索，从而提高检索效率。

`Scan Context`算法的详细流程如下图所示，可以发现`Ring Key`描述符检索`kdtree`可以快速得到最相似的`PC`描述符。然后使用上面给出的`PC`描述子距离公式计算点云相似程度。

<div align='center'>
    <image src="../images/process.png" alt="`Scan Context`算法流程" width=800px/>
</div>


### 1.3 `Scan Context`源码拓展内容

从`Scan Context`[代码仓库](https://github.com/gisbi-kim/scancontext_tro)给出的`C++`代码可以看出，代码中针对`PC`描述符相似度计算过程做了一定优化。代码中引入了`Sector Key`行向量描述符来**快速计算出$n^*$大小**。`Sector Key`行向量计算过程可由下面公式描述：

$$
\begin{align}
    &k_s&=&[\phi(c_1),...,\phi(c_{N_s})]\\
    &\phi(c_i)&=&\frac{||c_i||_1}{N_r}
\end{align}
$$

其中，

- $c_i$为`PC`描述子矩阵第$i$列，代表一列描述子信息；
- $\phi(c_i)$为`Sector Key`的列编码公式，由平均`L1`范数描述，并除以`N_r`，得到列编码；
- $k_s$为`Sector Key`，其为所有列得到的计算结果的集合。

可以这么理解，`Sector Key`可以描述以雷达坐标系为中心的径向方向上的点云特征，可以由下面的公式计算两个`Sector Key`的相似程度，并计算出对应的$n^*$，然后使用$n^*$邻域作为`PC`描述符相似度的计算区间，可以大大减少$n$的取值范围，从而提升`PC`描述符的计算效率。

$$
\begin{align}
    &d(k_s^{q},k_s^c)&=&1-\frac{k_s^q\cdot k_s^c}{||k_s^q||||k_s^c||}\\
    &D(k_s^q,k_s^c)&=&\min_{n\in [N_s]}{d(k_s^q,{k_s}_n^c)}\\
    &n^*&=&\arg \min_{n\in [N_s]} {d(k_s^q,{k_s}_n^c)}
\end{align}
$$

其中，

- $k_s^{q}$为当前点云的`Sector Key`向量；
- $k_s^c$为数据库中点云的`Sector Key`向量；
- ${k_s}_n^c$为左移`n`列时对应的`Sector Key`向量；
- $d(k_s^{q},k_s^c)$为`Sector Key`向量距离；
- $D(k_s^{q},k_s^c)$为最小`Sector Key`向量距离；
- $n^*$为最小距离对应列左移索引。

## 2. `Scan Context++`

### 2.1 `CC`描述符构建

`scan context++`算法除了使用`PC`描述符以外，还引入了`CC`描述符，`CC`描述符的构建过程如下图所示。与`PC`描述符构建的极坐标系不同，`CC`描述符构建的坐标系为笛卡尔坐标系，并以当前雷达坐标系为中心，在给定`bin`尺寸后，可构建由矩阵描述的`Cart Context`描述符。

<div align='center'>
    <image src="../images/scan_context++.png" alt="PC和CC描述符" width=800px/>
</div>

`Cart Context`描述符的距离计算过程与`PC`描述符距离计算过程一致，并且`Scan Context++`算法在论文中提到了`Scan Context`代码优化中的`Sector key`，从而加速描述符距离计算过程。

### 2.2 虚拟描述符增强

<div align='center'>
    <image src="../images/argument.png" alt="PC和CC描述符" width=800px/>
</div>

在`Scan Context`算法中，提到了`Root Shif`的概念，即为了增强`PC`描述符的平移不变性，将激光雷达点云中心移动到其他位置重新计算描述符，从而得到一个虚拟描述符，并虚拟描述符对应的平移变换，在计算相似度时，除了和原`PC`描述符计算距离外，还需要和虚拟描述符计算距离，取最小值作为最终的相似度。

`Scan Context++`算法补充了`CC`描述符增强概念，为了提高`CC`描述符的旋转不变性，构造旋转180度的`CC`虚拟描述符。反映在`CC`描述符矩阵上为上下和左右倒置。

