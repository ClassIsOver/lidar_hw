### 12.coding
可能需要补充的知识：
- ROS的subscriber和publisher的用法
- ROS的message filter的基本用法
- ROS的面对对象OOP编程

```c
    scan_filter_->registerCallback(boost::bind(&Scan2::scanCallBack, this, _1));
```
代码中的这里，是把函数的地址绑定到callback上
不了解boost::bind的话，可以参考:
http://blog.think-async.com/2010/04/bind-illustrated.html

### 3. 通过互联网总结学习线性方程组 Ax=b 的求解方法，回答以下问题:(2 分) 
#### (1)对于该类问题，你都知道哪几种求解方法? 
首先线性方程组的解有可能有三种情况：有无数解，有唯一解，无解。

LU分解，QR分解，SVD法，特征值分解法等。
- 最直接的最小二乘解法，通解：$x^* = (A^TA)^{-1}b$
- LU分解，$A=LDU$
又称为LR分解，将正方形矩阵分解为下三角矩阵L，乘以对角矩阵D，乘以上三角矩阵U。
下三角矩阵只有主对角线和下侧有值。
- LDLT分解，$A=LDL^T$
A为对称矩阵，且它的任意主子阵均不为0时，可以有唯一的分解。
- LLT分解，$A=LL^T = R^TR$
LDLT分解的特殊形式，又叫做Cholesky分解。要求A为正定的对称矩阵。R为上三角矩阵。
正定矩阵要求矩阵的所有特征值大于0。
- QR分解，$a=QR$
要求A是实数矩阵？，且n个列线性无关。Q是正交矩阵$QQ^T=I$，R是上三角矩阵。
- SDV奇异值分解，$A=UDV^T$
其中D对角线上的是A矩阵的全部非零奇异值。
https://zhuanlan.zhihu.com/p/57803955

(2)各方法的优缺点有哪些?分别在什么条件下较常被使用?


4. 简答题，开放性答案:设计里程计与激光雷达外参标定方法。(2 分) 
我们一般把传感器内自身要调节的参数称为内参，比如前面作业中里程计模型的两轮间距与两个轮子的半径。把传感器之间的信息称为外参，比如里程计与激光雷达之间的时间延迟，位姿变换等。请你选用直接线性方法或基于模型的方法，设计一套激光雷达与里程计外参的标定方法，并回答以下问题: 
(1)你设计的方法是否存在某些假设?基于这些假设下的标定观测值和预测值分别是什么? 
(2)如何构建你的最小二乘方程组求解该外参?