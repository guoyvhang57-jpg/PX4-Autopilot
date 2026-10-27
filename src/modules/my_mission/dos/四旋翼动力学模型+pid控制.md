# 最简化四旋翼动力学模型

## 1. 基本假设

- 四旋翼视为刚体，质量均匀，转动惯量为常量。
- 机体对称，质心位于几何中心。
- 忽略空气阻力、电机动态、桨叶挥舞等次要因素。
- 推力沿机体 z 轴方向，且与电机转速平方成正比。
- 采用“X”型布局：四个电机装在机臂末端，位于 ±45° 的四个角上，机头方向落在两个前电机之间。若为“+”型，仅控制分配矩阵不同（见第 4 节末的对比）。

## 2. 坐标系与状态量

- 世界坐标系 \(E\)：惯性系，z 轴竖直向上。
- 机体坐标系 \(B\)：原点在质心，z 轴指向机体上方。

状态量：

| 符号 | 含义 | 维度 |
|---|---|---|
| \(\boldsymbol{p}=[x,y,z]^T\) | 世界系下位置 | 3 |
| \(\boldsymbol{v}=[\dot x,\dot y,\dot z]^T\) | 世界系下速度 | 3 |
| \(\boldsymbol{\eta}=[\phi,\theta,\psi]^T\) | 欧拉角：滚转、俯仰、偏航 | 3 |
| \(\boldsymbol{\omega}=[p,q,r]^T\) | 机体系下角速度 | 3 |

## 3. 最简化动力学方程

### 3.1 平动方程

\[
\dot{\boldsymbol{p}}=\boldsymbol{v}
\]

\[
m\dot{\boldsymbol{v}}
=
\boldsymbol{R}(\boldsymbol{\eta})
\begin{bmatrix}
0\\
0\\
T
\end{bmatrix}
-
\begin{bmatrix}
0\\
0\\
mg
\end{bmatrix}
\]

即：

\[
\begin{bmatrix}
\ddot x\\
\ddot y\\
\ddot z
\end{bmatrix}
=
\frac{T}{m}\boldsymbol{R}(\boldsymbol{\eta})
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
-
\begin{bmatrix}
0\\
0\\
g
\end{bmatrix}
\]

其中 \(\boldsymbol{R}(\boldsymbol{\eta})\) 为机体坐标系到世界坐标系的旋转矩阵：

\[
\boldsymbol{R}(\boldsymbol{\eta})
=
\begin{bmatrix}
c\psi c\theta &
c\psi s\theta s\phi - s\psi c\phi &
c\psi s\theta c\phi + s\psi s\phi\\
s\psi c\theta &
s\psi s\theta s\phi + c\psi c\phi &
s\psi s\theta c\phi - c\psi s\phi\\
-s\theta &
c\theta s\phi &
c\theta c\phi
\end{bmatrix}
\]

式中 \(c=\cos(\cdot)\)，\(s=\sin(\cdot)\)。

### 3.2 转动方程

\[
\boldsymbol{J}\dot{\boldsymbol{\omega}}
+
\boldsymbol{\omega}\times(\boldsymbol{J}\boldsymbol{\omega})
=
\boldsymbol{\tau}
\]

对于对称机体，\(\boldsymbol{J}=\mathrm{diag}(J_x,J_y,J_z)\)，展开为：

\[
\begin{aligned}
J_x\dot p &= (J_y-J_z)qr+\tau_\phi\\
J_y\dot q &= (J_z-J_x)pr+\tau_\theta\\
J_z\dot r &= (J_x-J_y)pq+\tau_\psi
\end{aligned}
\]

该部分的p q r实际是角速度计测得的在机体坐标系下的三轴角速度矢量，不能直接积分得到角度，因为角度的合成要考虑不同时刻角速度的顺序，积分直接相加丢失顺序信息，所以这个公式一般用于算已知期望角加速度（角速度环生成的）得到力矩，对于想通过角速度得到欧拉角（给人看的或做角度环控制律设计的）用四元数（主流）或把一个小时刻的角速度转变成旋转矩阵项相乘得到

### 3.3 欧拉角运动学

\[
\dot{\boldsymbol{\eta}}
=
\boldsymbol{W}(\boldsymbol{\eta})\boldsymbol{\omega}
\]

\[
\boldsymbol{W}(\boldsymbol{\eta})
=
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta\\
0 & \cos\phi & -\sin\phi\\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}
\]

展开为：

\[
\begin{aligned}
\dot\phi &= p+q\sin\phi\tan\theta+r\cos\phi\tan\theta\\
\dot\theta &= q\cos\phi-r\sin\phi\\
\dot\psi &= q\sin\phi/\cos\theta+r\cos\phi/\cos\theta
\end{aligned}
\]
该部分其实是通过角速度来得到欧拉角的，但px4等实际工程为了避免万象节锁死，一般用四元数法。

## 4. 控制输入与“X”型布局分配

### 4.1 电机的几何位置

四个电机在机臂末端、位于 ±45° 的四个角上。按顺时针编号，**对角线上的两个电机同向转，相邻的逆向转**：

```
                      +x（机头）
                           ↑
        3 ●                │                ● 1
      (左前, CW)           │           (右前, CCW)
             \             │             /
              \            │            /
               \           │           /
      ──────────\──────────┼──────────/──────────→ +y（右）
                /          │          \
               /           │           \
        4 ●   /            │            \   ● 2
             / 		   |             \
      (右后, CW)           │           (左后, CCW)
```

| 电机 | 位置 \((x_i,y_i)\)（机体系，x 前、y 右） | 转向（俯视） | 本机对应 |
|---|---|---|---|
| 1 | \((+d,+d)\) 右前 | CCW | `CA_ROTOR0` |
| 2 | \((-d,-d)\) 左后 | CCW | `CA_ROTOR1` |
| 3 | \((+d,-d)\) 左前 | CW | `CA_ROTOR2` |
| 4 | \((-d,+d)\) 右后 | CW | `CA_ROTOR3` |

其中：

- \(l\)：电机到质心的距离（对角线长度），单位 m；
- \(d=l/\sqrt2\)：电机在 x 轴和 y 轴上的投影距离，也就是**滚转/俯仰的力臂**，单位 m。

### 4.2 力和力矩

记第 \(i\) 个电机推力 \(U_i=k\omega_i^2\)，方向沿机体 \(+z\)。由 \(\boldsymbol{\tau}=\sum\boldsymbol{r}_i\times\boldsymbol{F}_i\)：

\[
\begin{aligned}
T &= U_1+U_2+U_3+U_4\\
\tau_\phi &= d\,(U_1-U_2-U_3+U_4)\\
\tau_\theta &= d\,(-U_1+U_2-U_3+U_4)\\
\tau_\psi &= b\,(-\omega_1^2-\omega_2^2+\omega_3^2+\omega_4^2)
\end{aligned}
\]

| 控制量 | 表达式 | 含义 |
|---|---|---|
| \(T\) | \(k(\omega_1^2+\omega_2^2+\omega_3^2+\omega_4^2)\) | 总升力 |
| \(\tau_\phi\) | \(kd(\omega_1^2-\omega_2^2-\omega_3^2+\omega_4^2)\) | 滚转力矩 |
| \(\tau_\theta\) | \(kd(-\omega_1^2+\omega_2^2-\omega_3^2+\omega_4^2)\) | 俯仰力矩 |
| \(\tau_\psi\) | \(b(-\omega_1^2-\omega_2^2+\omega_3^2+\omega_4^2)\) | 偏航力矩 |

**物理直觉**：

- 滚转：1、4 号（右侧）加大、2、3 号（左侧）减小 → 右翼抬高（\(	au_\phi>0\)），机体向左倾
- 俯仰：1、3 号（前侧）加大、2、4 号（后侧）减小 → 后侧升力小 → 抬头（\(	au_\theta<0\)）
- 偏航：1、2 号（CCW）加大、3、4 号（CW）减小 → 反扭矩不平衡 → 机头顺时针转（\(	au_\psi<0\)）

### 4.3 写入矩阵形式

\[
\begin{bmatrix}
T\\ \tau_\phi\\ \tau_\theta\\ \tau_\psi
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
 k &  k &  k &  k\\
 kd & -kd & -kd &  kd\\
-kd &  kd & -kd &  kd\\
-b & -b &  b &  b
\end{bmatrix}
}_{\boldsymbol{M}}
\begin{bmatrix}
\omega_1^2\\ \omega_2^2\\ \omega_3^2\\ \omega_4^2
\end{bmatrix}
\]

\(\boldsymbol{M}\) 就是**控制分配矩阵**（PX4 里叫 effectiveness matrix，本质是纯几何量）。它满足 \(\boldsymbol{M}\boldsymbol{M}^T=4\,\mathrm{diag}(k^2,k^2d^2,k^2d^2,b^2)\)，所以**逆矩阵就是转置乘系数**，控制分配（给定期望的 \(T,\boldsymbol{\tau}\) 反求转速）可以解析写出来：

\[
\begin{aligned}
\omega_1^2 &= \tfrac14\left(\tfrac{T}{k}+\tfrac{\tau_\phi}{kd}-\tfrac{\tau_\theta}{kd}-\tfrac{\tau_\psi}{b}\right)\\
\omega_2^2 &= \tfrac14\left(\tfrac{T}{k}-\tfrac{\tau_\phi}{kd}+\tfrac{\tau_\theta}{kd}-\tfrac{\tau_\psi}{b}\right)\\
\omega_3^2 &= \tfrac14\left(\tfrac{T}{k}-\tfrac{\tau_\phi}{kd}-\tfrac{\tau_\theta}{kd}+\tfrac{\tau_\psi}{b}\right)\\
\omega_4^2 &= \tfrac14\left(\tfrac{T}{k}+\tfrac{\tau_\phi}{kd}+\tfrac{\tau_\theta}{kd}+\tfrac{\tau_\psi}{b}\right)
\end{aligned}
\]

> 这就是飞控里“混控（mixer）”要做的事。真实飞控还要处理 \(\omega_i^2<0\) 的饱和（某个电机算出来要反转），以及四个电机的推力系数不一致（换桨/电机老化）时的重新分配。



## 5. 符号含义汇总

| 符号 | 含义 | 单位 |
|---|---|---|
| \(m\) | 机体总质量 | kg |
| \(g\) | 重力加速度，约 \(9.81\) | m/s² |
| \(\boldsymbol{p}\) | 世界系位置 | m |
| \(\boldsymbol{v}\) | 世界系速度 | m/s |
| \(\boldsymbol{\eta}=(\phi,\theta,\psi)\) | 滚转角、俯仰角、偏航角 | rad |
| \(\boldsymbol{\omega}=(p,q,r)\) | 机体系角速度 | rad/s |
| \(\boldsymbol{R}(\boldsymbol{\eta})\) | 机体到世界的旋转矩阵 | — |
| \(\boldsymbol{W}(\boldsymbol{\eta})\) | 欧拉角运动学矩阵 | — |
| \(\boldsymbol{J}=\mathrm{diag}(J_x,J_y,J_z)\) | 机体转动惯量 | kg·m² |
| \(T\) | 总推力，沿机体 z 轴 | N |
| \(\boldsymbol{\tau}=(\tau_\phi,\tau_\theta,\tau_\psi)\) | 三轴控制力矩 | N·m |
| \(\omega_i\) | 第 \(i\) 个电机转速 | rad/s |
| \(U_i=k\omega_i^2\) | 第 \(i\) 个电机的推力 | N |
| \(k\) | 推力系数 | N·s²/rad² |
| \(b\) | 反扭矩系数 | N·m·s²/rad² |
| \(l\) | 电机到质心的距离（对角线长度） | m |
| \(d=l/\sqrt2\) | 电机在 x / y 轴上的投影，即滚转/俯仰力臂 | m |
| \(\boldsymbol{M}\) | 控制分配矩阵（4×4） | — |

## 6. 完整状态方程汇总

\[
\boxed{
\begin{aligned}
\dot{\boldsymbol{p}} &= \boldsymbol{v}\\
\dot{\boldsymbol{v}} &= \frac{T}{m}\boldsymbol{R}(\boldsymbol{\eta})\boldsymbol{e}_3-g\boldsymbol{e}_3\\
\dot{\boldsymbol{\eta}} &= \boldsymbol{W}(\boldsymbol{\eta})\boldsymbol{\omega}\\
\dot{\boldsymbol{\omega}} &= \boldsymbol{J}^{-1}\left(\boldsymbol{\tau}-\boldsymbol{\omega}\times\boldsymbol{J}\boldsymbol{\omega}\right)
\end{aligned}
}
\]

其中 \(\boldsymbol{e}_3=[0,0,1]^T\)。

- 输入：\(T,\tau_\phi,\tau_\theta,\tau_\psi\)，或四个电机转速 \(\omega_1,\omega_2,\omega_3,\omega_4\)。
- 输出：位置 \(\boldsymbol{p}\) 与姿态 \(\boldsymbol{\eta}\)。

## 7. 小角度线性化最简模型

在悬停附近，假设 \(\phi,\theta,\psi\) 较小，且 \(T\approx mg\)，可得：

\[
\begin{aligned}
\ddot x &\approx \frac{T}{m}\theta\\
\ddot y &\approx -\frac{T}{m}\phi\\
\ddot z &\approx \frac{T}{m}-g\\
J_x\ddot\phi &\approx \tau_\phi\\
J_y\ddot\theta &\approx \tau_\theta\\
J_z\ddot\psi &\approx \tau_\psi
\end{aligned}
\]

该线性化模型常用于四旋翼姿态控制与位置控制设计。
