# homework 1

#### 阅读《鸟哥的 Linux 私房菜》自学前三部分内容
#### (1）列举三个你常用的 Linux 命令，并说明他们的功能。
touch filename 新建文件
mkdir dirname  新建文件夹
ls      列举当前目录下的文件
sudo    暂时获得管理员权限并运行某命令


#### (2）一句话简要介绍 Vim 的功能，如何在 Vim 中进行插入和删除，如何保存并退出 Vim？
Vim是一种具有编程能力的文本编辑器。
插入：在一般模式下，输入i,o,a进入编辑模式，然后可以在光标处插入字符。
删除：在编辑模式下用backpace或delete删除，或按esc进入一般模式，使用x,X删除字符，使用nx删除n个字符。
保存并退出：一般模式下按“:wq”

#### (3）列举两种常用的 Linux 压缩和解压缩命令。
压缩指令：gzip, bzip2
解压缩指令：gzip -d, bzip2 -d

2. 了解ROS:观看ROS免费公开课或前往ROS官网学习官方教程，安装好ROS，提供运行小海龟跑的截图;(3 分）
![小海龟](turtle.jpg)

3. 学习机器人姿态描述入门材料，完成坐标转换推导;(3 分）

#### 设机器人的世界坐标为 $x_a, y_a$，其相对于世界坐标系的方向为 $θ_a$(右手坐标系）。假设机器人旁边有一物体在世界坐标系下的位姿为$(x_b, y_b, θ_b)$,请问: 
####(1）该物体相对于机器人的位置和朝向是什么，即该物体在当前机器人坐标系下的位姿是多少? 

我们将机器人坐标系记为A，世界坐标系记为W，物体坐标系记为B，则已知机器人坐标系到世界坐标系的坐标转换矩阵$T^W_A$和物体坐标系到世界坐标系的坐标转换矩阵$T^W_B$：
```math
\begin{aligned}
T^W_A = 
\begin{bmatrix}
 cos\theta_a & -sin\theta_a & x_a \\
 sin\theta_a &  cos\theta_a & y_a \\ 0 & 0 & 1
\end{bmatrix}
\quad
T^W_B = 
\begin{bmatrix}
 cos\theta_b & -sin\theta_b & x_b \\
 sin\theta_b &  cos\theta_b & y_b \\ 0 & 0 & 1
\end{bmatrix}
\end{aligned}\tag{1}
```

则物体坐标系到机器人坐标系的坐标转换矩阵$T^A_B$为：
```math
\begin{aligned}
T^A_B &= T^A_W T^W_B = (T^W_A)^{-1} T^W_B\\
&= 
\begin{bmatrix}
 cos\theta_a & -sin\theta_a & x_a \\
 sin\theta_a &  cos\theta_a & y_a \\ 0 & 0 & 1
\end{bmatrix}^{-1}
\begin{bmatrix}
 cos\theta_b & -sin\theta_b & x_b \\
 sin\theta_b &  cos\theta_b & y_b \\ 0 & 0 & 1
\end{bmatrix}\\
&= 
\begin{bmatrix}
 cos\theta_a &  sin\theta_a & 
 -cos\theta_a x_a - sin\theta_a y_a\\
 -sin\theta_a &  cos\theta_a & 
 sin\theta_a x_a - cos\theta_a y_a\\ 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
 cos\theta_b & -sin\theta_b & x_b \\
 sin\theta_b &  cos\theta_b & y_b \\ 0 & 0 & 1
\end{bmatrix}
\end{aligned}\tag{2}
```
计算可得：
```math
\begin{aligned}
T^A_B(0,0) &= cos\theta_a cos\theta_b + sin\theta_a sin\theta_b = cos(\theta_b - \theta_a)\\
T^A_B(0,1) &= -cos\theta_a sin\theta_b + sin\theta_a cos\theta_b = -sin(\theta_b - \theta_a)\\
T^A_B(0,2) &= cos\theta_a x_b + sin\theta_a y_b - cos\theta_a x_a - sin\theta_a y_a\\ &= cos\theta_a(x_b-x_a) + sin\theta_a(y_b-y_a)\\

T^A_B(1,0) &= -sin\theta_a cos\theta_b + cos\theta_a sin\theta_b = sin(\theta_b - \theta_a)\\
T^A_B(1,1) &= T^A_B(0,0)\\
T^A_B(1,2) &= -sin\theta_a x_b + cos\theta_a y_b + sin\theta_a x_a - cos\theta_a y_a \\ &= -sin\theta_a(x_b-x_a) + cos\theta_a(y_b-y_a)\\

T^A_B(2,0) &= 0\\
T^A_B(2,1) &= 0\\
T^A_B(2,2) &= 1
\end{aligned}\tag{3}
```
记物体在机器人坐标系的位姿为$p^A_B(x^a_b, y^a_b, θ^a_b)$，则其坐标转换矩阵
```math
\begin{aligned}
T^A_B =
\begin{bmatrix}
 cos\theta^a_b & -sin\theta^a_b & x^a_b \\
 sin\theta^a_b &  cos\theta^a_b & y^a_b \\ 0 & 0 & 1
\end{bmatrix}
\end{aligned}\tag{4}
```
将公式(3)的结果带入公式(4)可得物体在机器人坐标系的位姿：
```math
\begin{aligned}
x^a_b &= T^A_B(0,2) = cos\theta_a(x_b-x_a) + sin\theta_a(y_b-y_a)\\
y^a_b &= T^A_B(1,2) = -sin\theta_a(x_b-x_a) + cos\theta_a(y_b-y_a)\\
\theta^a_b &= atan2(T^A_B(1,0), T^A_B(0,0)) = \theta_b - \theta_a
\end{aligned}\tag{5}
```
####(2）机器人此时朝它的正前方(机器人坐标系 X 轴）行进了 d 距离，然后又转了 $θ_d$ 角，请问物体此时在这一时刻机器人坐标系下的位姿是多少?

根据(1)中的计算，若记$^Wt^W_A=[x_a, x_b]^T$

记此时的新的机器人坐标系为D，坐标系D在坐标系A中的位姿为$p^A_D(d,0,\theta_d)$，记此时机器人坐标系D下的物体B位姿为$p^D_B(x^d_b, y^d_b, \theta^d_b)$。

```math
\begin{aligned}
T^D_B &= T^D_A T^A_B = (T^A_D)^{-1} T^A_B\\
&= 
\begin{bmatrix}
 cos\theta_d & -sin\theta_d & d \\
 sin\theta_d &  cos\theta_d & 0 \\ 0 & 0 & 1
\end{bmatrix}^{-1}
\begin{bmatrix}
 cos\theta^a_b & -sin\theta^a_b & x^a_b \\
 sin\theta^a_b &  cos\theta^a_b & y^a_b \\ 0 & 0 & 1
\end{bmatrix}\\
&= 
\begin{bmatrix}
 cos\theta_d &  sin\theta_d & -cos\theta_d d\\
 -sin\theta_d &  cos\theta_d & sin\theta_d d\\ 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
 cos\theta^a_b & -sin\theta^a_b & x^a_b \\
 sin\theta^a_b &  cos\theta^a_b & y^a_b \\ 0 & 0 & 1
\end{bmatrix}
\end{aligned}\tag{6}
```
类推(1)的结果可得：
```math
\begin{aligned}
x^d_b &= T^D_B(0,2) = cos\theta_d(x^a_b - d) + sin\theta_d y^a_b\\
&= cos\theta_d\{[cos\theta_a(x_b-x_a) + sin\theta_a(y_b-y_a)] - d\} \\&+ sin\theta_d [-sin\theta_a(x_b-x_a) + cos\theta_a(y_b-y_a)]\\
y^d_b &= T^D_B(1,2) = -sin\theta_d(x^a_b - d) + cos\theta_d y^a_b\\
&= -sin\theta_d\{[cos\theta_a(x_b-x_a) + sin\theta_a(y_b-y_a)] - d\} \\&+ cos\theta_d [-sin\theta_a(x_b-x_a) + cos\theta_a(y_b-y_a)]\\
\theta^d_b &= \theta^a_b - \theta_d = \theta_b - \theta_a - \theta_d
\end{aligned}\tag{7}
```

