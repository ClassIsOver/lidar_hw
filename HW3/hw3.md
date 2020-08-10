### 1 补充代码
```c
void Lidar_MotionCalibration(
            tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> segment_start_pose,
            tf::Stamped<tf::Pose> segment_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            const int startIndex,
            const int& beam_number)
    {
       //TODO
       tf::Vector3 origin_begin = segment_start_pose.getOrigin();
       tf::Vector3 origin_end   = segment_end_pose.getOrigin();
       tf::Quaternion rotation_begin = segment_start_pose.getRotation();
       tf::Quaternion rotation_end   = segment_end_pose.getRotation();

       for (int i = 0; i <= beam_number; i++){
         double ratio = static_cast<double>(i) / static_cast<double>(beam_number);
         tf::Vector3 origin = origin_begin.lerp(origin_end, ratio);
         tf::Quaternion rotation = rotation_begin.slerp(rotation_end, ratio);
         tf::Pose pose(rotation, origin);

         int pt_id = i + startIndex;
         tf::Vector3 pt(ranges[pt_id] * cos(angles[pt_id]),
                        ranges[pt_id] * sin(angles[pt_id]), .0);
         tf::Vector3 pt_in_base_frame = frame_base_pose.inverse() * pose * pt;

         // Overwite with motion compensated measurement
         ranges[pt_id] =
             sqrt(pt_in_base_frame.getX() * pt_in_base_frame.getX() +
                  pt_in_base_frame.getY() * pt_in_base_frame.getY());
         angles[pt_id] =
             atan2(pt_in_base_frame.getY(), pt_in_base_frame.getX());
       }
       //end of TODO
    }
```

### 2.推导并证明已知对应点的 ICP 求解方法
认为对于点集 $p=\{p_i\}; i = 1,2,...,N$ 和 $p'=\{p'_i\}; i = 1,2,...,N$，存在变换的最优的 $\hat R, \hat t$ 使目标函数最小：
```math
E(R,t) = \frac{1}{N} \sum_i^N {|| p'_i - (Rp_i + t)||^2 }
```

认为此时两个点集的中心 $\bar p$ 和 $\bar p'$ 满足 $\bar p' - (R\bar p + t) = 0$

定义 $q_i = p_i - \bar p$，$q'_i = p'_i - \bar p'$。则有：
```math
\begin{aligned}
E(R,t) 
&= \frac{1}{N} \sum_i^N {|| q'_i + \bar p' - [R(q_i + \bar p) + t]||^2 }\\
&= \frac{1}{N} \sum_i^N {|| q'_i - Rq_i + \bar p' - (R \bar p + t)||^2 }\\
&= \frac{1}{N} \sum_i^N {|| q'_i - Rq_i||^2 }
\end{aligned}
```

此时目标函数简化为只与 $R$ 有关。求解出最优的 $\hat R$ 后可得 $\hat t = \bar p' - \hat R\bar p$

继续展开目标函数，注意对 $1\times 1$ 矩阵，转置和它自己等价。
```math
\begin{aligned}
E(R,t) 
&= \frac{1}{N} \sum_i^N {|| q'_i - Rq_i||^2 } \\
&= \frac{1}{N} \sum_i^N {(q'_i - Rq_i)^T (q'_i - Rq_i)} \\ 
&= \frac{1}{N} \sum_i^N {(q'^T_i - q^T_i R^T) (q'_i - Rq_i)} \\ 
&= \frac{1}{N} \sum_i^N {q'^T_i q'_i +  q^T_i R^T Rq_i - q'^T_i Rq_i - q^T_i R^T q'_i}\\
&= \frac{1}{N} \sum_i^N {q'^T_i q'_i +  q^T_i q_i - q'^T_i Rq_i - (q'^T_i Rq_i)^T}\\
&= \frac{1}{N} \sum_i^N {q'^T_i q'_i +  q^T_i q_i - 2q'^T_i Rq_i}
\end{aligned}
```
由上式
```math
\begin{aligned}
\mathop{\arg\min}_{R,t} E(R,t) 
&= \mathop{\arg\max}_{R} \sum_i^N {q'^T_i Rq_i}
\end{aligned}
```
定义 $H := \sum_i^N q_i q'^T_i$，则根据迹的定义，有：
```math
\begin{aligned}
\sum_i^N {q'^T_i Rq_i} &= Trace(\sum_i^N R q_iq'^T_i) \\
&= Trace(R \sum_i^N q_iq'^T_i) &= Trace(RH)
\end{aligned}
```
对H进行SVD分解：$H = U \Lambda V^T$。其中 $U,V$ 是正交矩阵，$\Lambda$ 是对角阵。

可以证明，对于任意正定矩阵 $AA^T$ 和任意正交矩阵 $B$ ，都满足 $Trace(AA^T) >= Trace (BAA^T)$ 

则如果矩阵 $VU^TH$ 是一个正定矩阵，对任意的正交矩阵 $B$ 都满足 $Trace(VU^T H) >= Trace (BVU^TH)$，那么最小二乘解 $\hat R = VU^T$。

通过下式计算可知，$VU^TH$ 是一个正定矩阵：
```math
\begin{aligned}
VU^T \ H &=VU^T \ U\Lambda V^T = V\Lambda V^T\\
\end{aligned}
```
所以ICP的最小二乘解为：
```math
\begin{aligned}
\hat R &= VU^T\\
\hat t &= \bar p' - \hat R\bar p
\end{aligned}
```
