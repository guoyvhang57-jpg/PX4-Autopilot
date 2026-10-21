# my_mission

Offboard 位置控制的最小任务示例：**起飞到指定高度 → 向前飞到目标点 → 悬停 → 降落 → 上锁**。

已接进构建（`modules__my_mission`，命令行名 `my_mission`）。

> 📖 **PX4 整体架构、话题分层、每个模式的控制链与准入条件、所有"判断条件"（能不能解锁 /
> 能不能进模式 / 能不能留在模式里）** —— 见 [PX4_架构与模式总说明.md](PX4_架构与模式总说明.md)。
> 觉得太密？先看 **[PX4_架构通俗版.md](PX4_架构通俗版.md)**（用比方讲同一件事）。
> 本 README 只讲 `my_mission` 自己。

## 文件

代码按职责拆成 5 块，依赖方向是单向的（`MyMission` → 其它，其它之间互不依赖）：

| 文件 | 作用 |
|---|---|
| `MyMission.cpp` / `MyMission.hpp` | **任务编排**：状态机 + 9 个状态函数。就是"什么时候做什么" |
| `MissionConfig.hpp` | `MYM_*` / `MYR_*` 参数的只读快照（POD），子组件不依赖参数框架 |
| `MissionOutput.cpp` / `.hpp` | **所有出站 topic**（`offboard_control_mode` / `trajectory_setpoint` / `vehicle_command`）。只有它能发 topic |
| `TransitLeg.cpp` / `.hpp` | "用 PX4 的 `PositionSmoothing` 飞到一个定点"，`FORWARD` 用它 |
| `ExternalParams.cpp` / `.hpp` | 从别人的参数抄阈值（`MPC_*` / `MAN_DEADZONE` / `LNDMC_Z_VEL_MAX`） |
| `TakeoverMonitor.cpp` / `.hpp` | 接管安全：飞出 Offboard 时油门没回中会掉高度 |
| `docs/offboard.md` | Offboard 是什么、三条 topic 怎么配合、飞手操作顺序（原来压在 `MyMission.cpp` 开头的 144 行） |
| `my_mission_params.yaml` | `MYM_*` / `MYR_*` 参数定义 |
| `Kconfig` | `MODULES_MY_MISSION` |
| `CMakeLists.txt` | `px4_add_module` |
| `PX4_架构通俗版.md` | **人话版** PX4 架构（一个杆量怎么变成电机转速；含本项目踩过的坑） |
| `PX4_控制链逐层拆解.md` | **逐层拆解**控制链（第 1 层已完整，含算得通的数字例子） |
| `PX4_意图与决策逐层拆解.md` | **逐层拆解**意图路（阶段 0~6：从遥控输入到便签写出来） |
| `PX4_架构与模式总说明.md` | 详细版 PX4 架构（参数名、源码位置、准入条件全文） |
| `my_mission_异常处理手册.md` | **异常处理手册**：出情况时飞控和本模块各自怎么处理（三栏对照） |
| `Commander_run_逐段说明.md` | **`Commander::run()` 逐段说明**：28 步的读/写/发布、实现方式、设计理由 |

## 状态机

```
WAIT ──手势解锁 + 进 Offboard──>
       ├─ 离地 + 离 home > MYM_ARRIVE ──> TILT（手动进场 = 拉绳测试，无开关）
       └─ 地面上 ──> TAKEOFF ──到达 MYM_ALT──>
                    ├─ MYM_FWD_DIST > 0 ──> FORWARD ──到位──> HOVER
                    └─ MYM_FWD_DIST = 0 ───────────────────> HOVER
     ──MYM_HOVER 秒──> RETURN（对准 home -> 飞回 -> 转回初始航向）──> LAND
     ──> DISARM ──> DONE ──(已上锁)──> WAIT（可以再跑一遍）
```

**所有位置参考都统一到 home**（PX4 `home_position`，**解锁那一刻**的位置，每次解锁重设）：

| 用途 | 参考 |
|---|---|
| `TAKEOFF` 起飞点 | home 的 x/y |
| `TAKEOFF` 目标高度 | `home.z − MYM_ALT`（`home.z` 就是那块地的地面） |
| 前飞段高度 | `home.z − max(MYM_ALT, MYM_ALT_HIGH)`，斜着爬 |
| `FORWARD` 终点 | home 的 x/y + `MYM_FWD_DIST` |
| 手动进场判据 | 到 home 的三维距离 |
| `RETURN` 目标 | home 的 x/y + 当前高度 |
| Offboard 下降目标 | `home.z` |
| 航向基准 | `home.yaw` —— 解锁时的机头方向 |

`home.yaw` 是整趟任务的航向基准：`MYM_FWD_DIR` 相对它、`TILT` 的前馈沿它、
`RETURN` 最后把机头转回它。起飞阶段 `_yaw_sp` 也设成它，否则"基准"和"实际机头"
会差一个角，前飞就变成斜着飞。

`home_position` 无效时全部退回"当前位置 + 此刻的机头方向 + 锁存的地面"，
行为和不加这套逻辑时一样。
唯一有意不以 home 为准的是**手动进场的拉绳点**——它的定义就是"飞手现在停在哪"。

**起飞到位之后走哪条路，在那一刻定死：**

| 配置 | 做什么 | 之后 |
|---|---|---|
| `MYM_FWD_DIST > 0` | `FORWARD` 飞到 home 前方 `MYM_FWD_DIST` 米 | `HOVER` |
| `MYM_FWD_DIST = 0` | 不飞 | 直接 `HOVER` |

**进场方式决定走哪条路，没有开关**：离地后切 Offboard = 拉绳测试；
在地面切 Offboard = 正常起飞流程。

实际的操作顺序（先切模式再解锁、手势怎么做、要设哪些参数）见下面
[操作顺序](#操作顺序先切-offboard再用手势解锁) 一节。

- `WAIT`：50 Hz 发 `offboard_control_mode{position=true}` + `trajectory_setpoint`（钉在当前位置）。
  这一步是让 commander 认为 "Offboard 可用"，**必须在切进 Offboard 之前就开始发**。
  进 `TAKEOFF` 时把当前航向锁进 `_takeoff_yaw`，后面 `FORWARD` 用它算"前方"。
- `TAKEOFF`：目标 = 进入点上方 `MYM_ALT`，NED 里就是 `z = 进入时 z - MYM_ALT`（向上为负）。
- `FORWARD`：飞向**前方点**。算法：
  终点 = **起飞点** 沿 `_takeoff_yaw + MYM_FWD_DIR` 前方 `MYM_FWD_DIST` 米，高度不变。
    - 机头单位向量在 NED 里是 `(cos ψ, sin ψ)`：ψ 的定义是"从北起顺时针为正"，
      而 NED 的 x 指北、y 指东，x→y 也是顺时针 —— **两个角度系统重合，不用转负号**。
    - 起点固定用起飞点（不是"当前位置"），所以同一个场地跑几次都落在同一点。
    - yaw 保持锁存值，飞机是**机头朝前平飞**过去，不是侧着飘过去。
    - **位置参考交给 PX4 的 `PositionSmoothing` 生成**，不是自己逐拍推进。
      `TAKEOFF` 只把终点存进 `_fwd_goal` / `_fwd_goal3`；进 `FORWARD` 时用飞机当前的
      位置/速度/加速度点亮平滑器，之后每拍由它给出位置、速度、**加速度**三条曲线。
      `MYM_FWD_SPD` 是它的巡航速度上限，`MYM_FWD_ACC` 是加速度上限。
      详见下面 [参考生成](#参考生成直接复用-px4-的-positionsmoothing)。
    - 到位判据 = **轨迹**贴住终点（`MYM_ARRIVE`）**且飞机**速度 < 0.3 m/s。
    - 超时 `MYM_FWD_TMO` → 直接转 `LAND`（和起飞超时同样的策略：不硬追）。
- `HOVER`：保持同一 setpoint `MYM_HOVER` 秒。
- **jerk 按需求姿态调度**：需求角离低头限幅远时用 `MYM_FWD_JERK_MAX`，
  从限幅的 80% 起线性降到 `MYM_FWD_JERK_MIN`。作用是把"姿态到限幅、前推力接手"
  这个交接过得柔和些——姿态和缺额出来的推力来自同一个加速度需求，全速撞边界
  会让两者同时阶跃。需求角取自上一拍发出的前馈（20 ms 前），不构成闭环。
  约束：一段航程的峰值加速度约 `sqrt(jerk × 峰值速度)`，只有超过
  `g·tan(限幅) = 2.63 m/s²` 时前推力才会触发。
- `TILT`（`MYR_ROPE_EN = 1` 才进）：目标留在起飞点上空，按 `MYR_PITCH_DOWN` /
  `MYR_FWD_THR` 发一条沿起飞航向的加速度前馈，**高度不变**，保持 `MYR_TILT_HOLD` 秒。
- `LAND`：`MYM_LAND_MODE=1` 发 `MAV_CMD_NAV_LAND` 交给 PX4 AUTO_LAND；`=0` 则纯 Offboard 把 z 以
  `MYM_DESC_RATE` 抬回地面参考。
- `DISARM`：发 `MAV_CMD_COMPONENT_ARM_DISARM(param1=0)`，等 `!armed`。
- 全程只要在起飞/前飞/绳段/低头/悬停阶段掉出 Offboard 或上锁，立即中止回 `DONE`。

> ⚠ **"前方"完全由航向估计决定**（GPS 双天线 / 磁罗盘 / EKF）。航向歪多少度，飞出去就歪多少度，
> 而且 `MYM_FWD_DIST` 越大偏得越远（3 m 上偏 30° ≈ 1.5 m）。
> 第一次试：`MYM_FWD_DIST` 给 1~2 m、`MYM_ALT` 给 ≥ 2 m，开阔场地，随时能切回 Position。

### 返航（`RETURN`）

悬停结束**不直接降落**：先飞回 **home**（px4 `home_position`，也就是**解锁那一刻**的
位置；每次解锁都会重设），到了再走 `LAND`。降落功能本身没动，`MYM_LAND_MODE` 两种
方式照旧，只是起点变成了 home。

分两步：

1. **原地对准航向** —— 算出到 home 的方位角，用它当 yaw 目标，位置保持在原地。
   两个条件**同时**满足才进下一步：

   | 条件 | 阈值 | 原因 |
   |---|---|---|
   | 航向误差 | 见 `kReturnYawErrMax` | 剩下那点误差在回程里会被 yaw 给定继续收敛 |
   | 机体 z 轴角速度 | 见 `kReturnYawRateMax` | **只判角度是不够的** —— 可能刚好扫到误差很小的瞬间但还在 60 °/s 地转，轨迹一开就是斜着出去 |

   **偏航给定本身是斜坡**：每拍从**实测机头方向**往前推 `kReturnYawSlew`（30 °/s，
   定死在 `MyMission.cpp` 顶部），不一步跳到目标。直接给目标航向的话，姿态环会按
   `MC_YAWRATE_MAX`（默认 200 °/s）猛甩，机体和绳都会被甩一下。
   所以判据里的"角速度够小"只有等斜坡走完、姿态跟上之后才会满足。

   距离小于 1 m 时整段跳过（没有方向可言）。对准超时（`MYM_FWD_TMO`）就地降落。
2. **飞回去** —— 复用 `FORWARD` 那套 `TransitLeg`：同一个 `PositionSmoothing`、
   同一套速度/加减速/jerk 限制（`MYM_FWD_SPD` / `MYM_FWD_ACC` / `MYM_FWD_JERK_*`）、
   同一个到位判据（`MYM_ARRIVE` + 速度 < 0.3 m/s）和超时（`MYM_FWD_TMO`）。

3. **在 home 转回初始航向** —— 到家后停住位置，把 yaw 以同样的 30 °/s 斜坡转回
   解锁时的机头方向（`_takeoff_yaw`），判据同上，然后**直接降落**。
   这样落地时机头方向和起飞前一致。转不动超过 10 s 就按当前航向降落。

**高度用当前值**，回家是同高度平飞，不是降到地面。

`home_position` 无效（没解锁过、话题没来）时就地降落并打一条 `PX4_WARN`，
不会乱飞。返航超时同样就地降落。

## 参数（`MYM_`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `MYM_ALT` | 1.0 m | 起飞目标高度 |
| `MYM_FWD_DIST` | 3.0 m | 到高度后沿机头方向再平飞多远。**0 = 跳过 FORWARD** |
| `MYM_FWD_DIR` | 0.0 deg | 前飞方向相对起飞航向的偏角：0 = 正前，+90 = 右，-90 = 左，180 = 后 |
| `MYM_FWD_SPD` | 0.5 m/s | 前飞巡航速度（对地）。位置目标是按这个速度**推**过去的，不是一步跳过去 |
| `MYM_FWD_ACC` | 1.0 m/s² | 前飞加/减速上限。**决定姿态变化大小**：平飞倾角 ≈ `atan(加速度/9.81)`，1.0 → ≈6° |
| `MYM_FWD_JERK_MAX` | 4.0 m/s³ | 远离限幅时的 jerk；≈5.84×值 度/秒 |
| `MYM_FWD_JERK_MIN` | 1.0 m/s³ | 贴着限幅时的 jerk |
| `MYM_FWD_TMO` | 20 s | 前飞超时 → 直接转降落。斜坡比阶跃慢，改小 `MYM_FWD_SPD` 时要一起放大 |
| `MYM_HOVER` | 5.0 s | 到达**前方点**后的悬停时间（绳开时用 `MYR_TILT_HOLD`，不用这个） |
| `MYM_LAND_MODE` | 1 | 0 = Offboard 下降，1 = PX4 AUTO_LAND |
| `MYM_DESC_RATE` | 0.4 m/s | Offboard 下降速率（仅 mode 0）。**会被强制抬到 ≥ `1.1 × LNDMC_Z_VEL_MAX`（0.275 m/s）**，否则 land detector 判不出"被指令下降"，`landed` 永远上不去 → 上不了锁 |
| `MYM_ALT_HIGH` | 0.0 m | 起飞后要爬到的高度（0 = 不额外爬升）；前飞时斜着爬 |
| `MYM_ARRIVE` | 0.20 m | 到位判定半径（同时要求速度 < 0.3 m/s） |
| `MYM_TKO_TMO` | 20 s | 起飞超时 → 直接转降落 |
| `MYM_LAND_TMO` | 40 s | 降落超时 → 转去上锁 |
| `MYM_AUTO_DISARM` | 1 | 着地后由本模块发 disarm |
| `MYM_DISARM_TMO` | 10 s | 上锁超时 |

## 拉绳测试（离地后切 Offboard）

**没有开关**，判定条件就是进场方式：**离地之后**才切进 Offboard，就认定是拉绳测试，
`WAIT` 直接进 `TILT`。条件三条：

1. 已经离地（land detector 说不在接触地面）
2. 到 **home** 的三维距离 > `MYM_ARRIVE`
3. 切 Offboard 那一刻

在地面 arm 之后再切 Offboard 的那一趟**永远不拉**，走 `TAKEOFF → FORWARD/HOVER →
RETURN → LAND`。

⚠ 这条规则是"离地就拉"：只要你在 POSCTL 里飞起来再切 Offboard，它就会低头。
不想拉就不要在空中切 Offboard。
`WAIT` 里一看到 Offboard 就判断"到 **PX4 `home_position`** 的三维距离"（那个点
在**解锁那一刻**设，每次解锁都重设，语义是"这一趟从哪起飞"），超过 `MYM_ARRIVE`
且**已经离地**，就跳过起飞直接进 `TILT`，三轴目标都取当前位置（不套 `MYM_ALT`）。

没收到 `home_position`（或它的 `valid_lpos` 为 false）时退回 EKF 本地原点，
也就是"这次上电在哪"，行为和不加这个功能时一致。日志里会写清楚用的是哪个：

```
my_mission: manual entry 8.32 m from home (4.87 m above ground) -> PULL 10.0 deg, 10.0 s
```

判据里的 `!landed` 是安全护栏：地上的飞机被人挪过位置时，离原点也会超过
`MYM_ARRIVE`，不加这条会坐在地上拉绳。

⚠ 手动进场这条路**落地就在拉绳的地方**，不返回原点（没有起飞点可回）。

```
TAKEOFF ──到达 MYM_ALT──> TILT ──MYR_TILT_HOLD 秒──> HOVER ──> LAND
```

`TILT` 做两件事：目标一直留在起飞点上空（位置环维持高度和水平位置），沿起飞航向
发一条加速度前馈，保持 `MYR_TILT_HOLD` 秒后按 2.5 m/s²/s 的斜坡退回、转悬停。

### 前馈由角度和油门反解

```
MYR_FWD_THR = 0  ->  a = g * tan(MYR_PITCH_DOWN)                 姿态 = 你给的角度，推进电机不转
MYR_FWD_THR > 0  ->  a = g * ( tan(上限) + MYR_FWD_THR / SC )    姿态 = 上限（15°），推进电机 = 你给的油门
```

上限 = `min(|MYM_PITCH_MIN|, MPC_TILTMAX_AIR)`，和位置环一致。这个式子是位置环
那套的逆运算（需求角 `atan(a/g)` → 夹在上限 → 缺额 `tan(需求)-tan(上限)` 乘 `SC`
就是归一化前推力），所以姿态和油门各自落在设定值上，不会互相干扰。

对表（上限 15°、SC 0.44、4 kg）：

| 配置 | a | 俯仰 | 推进电机 | 前向力 |
|---|---|---|---|---|
| `PITCH_DOWN=5, THR=0` | 0.86 m/s² | −5° | 0% | ≈3.4 N |
| `PITCH_DOWN=10, THR=0` | 1.73 m/s² | −10° | 0% | ≈6.9 N |
| `PITCH_DOWN=15, THR=0` | 2.63 m/s² | −15° | 0% | ≈10.5 N |
| `THR=0.2` | 7.09 m/s² | −15° | 20% | ≈28.7 N |
| `THR=0.5` | 13.8 m/s² | −15° | 50% | ≈55.9 N |

⚠ 给油门必须同时满足 `MYM_FWD_THR_EN = 1`（否则没人出力）和 `MYM_PITCH_MIN < 0`
（否则没有上限可反解）。

### 绳在仿真里，不在参数里

| 项 | 在哪 |
|---|---|
| 绳长 | SDF 的 `<rope_length>`，或启动时 `GZ_TETHER_LEN` |
| 锚点 | SDF 的 `<anchor_offset>`，gz 世界系 ENU，相对出生点的地面投影 |
| 刚度 / 阻尼 / 上限 | `<stiffness>` / `<damping>` / `<max_tension>` |

模块不知道绳有多长、拴在哪，也**不算**绷紧点。前提是摆成"飞机爬到 `MYM_ALT` 时
绳已经绷紧"，否则低头之后只会往前飞，测不到拉力。

**`GZ_TETHER=1` 不能漏**，没有它插件整个不工作。

⚠ 每次测试都要重启仿真：锚点在世界里固定，模型落地后停在哪、下一趟就从哪起飞，
同一个会话里跑第二趟几何已经完全不一样。

| 参数 | 默认 | 说明 |
|---|---|---|
| `MYR_PITCH_DOWN` | 15° | 目标低头角，0~15；**油门为 0 时才生效** |
| `MYR_TILT_HOLD` | 10 s | 拉多久 |
| `MYR_FWD_THR` | 0.00 | 前推力电机油门；一给，姿态就钉在 15° |

## 前推力辅助（谁允许、什么时候允许）

四个门槛，全真才出力：

| # | 门槛 | 谁说了算 |
|---|---|---|
| ① | `MYM_FWD_THR_EN != 0` | 用户参数，总开关 |
| ② | 在 Offboard 里 | `vehicle_control_mode.flag_control_offboard_enabled` |
| ③ | 阶段许可 | 本模块每拍发的 `my_mission_assist.enabled` |
| ④ | 需求角 > `MYM_PITCH_MIN` | `PositionControl` 算缺额，> 0 才有推力 |

③ 只在三个阶段为真：

- `TILT`（拉绳段）—— 整段
- `FORWARD`（去程）—— 直到本段的减速段闩锁
- `RETURN`（回程）—— 同样是长距离前飞，规则相同

其余状态（`TAKEOFF` / `HOVER` / `LAND` / `DISARM` / `DONE` / `WAIT`）一律为假。

**关的时刻**：控制器自己把前推力给成 0（`vehicle_attitude_setpoint.thrust_body[0] < 2%`）
**并且**轨迹已进入减速段（沿航向前馈 < −0.2），连续 0.3 s 才单向闩锁。这时缺额本来就是 0，
关掉辅助只是把闲置的夹紧放开，姿态不会跳。每段航程各有一份闩锁，`enterState` 清零。

**超时按"禁止"处理（fail-closed）。** 控制器 500 ms 收不到 `my_mission_assist` 就认为
辅助关闭。代价是这条路变成 `my_mission` 专用：用 MAVSDK / ROS 2 飞 Offboard 时拿不到
辅助（它们不发这个话题）。好处是模块崩了、被 `my_mission stop` 停了，推力电机立刻
回到"不参与"，不会在没人管的时候继续出力。

## 操作顺序：先切 Offboard，再用手势解锁

模块**不自己 arm、也不自己切模式**，它只检测 `armed && offboard && 位置有效`。
推荐顺序如下（先模式后解锁）：

| 步骤 | 做什么 | 为什么 |
|---|---|---|
| 1 | 等 EKF 位置有效 | position 型 Offboard 的硬前提（`offboardCheck.cpp:56`） |
| 2 | 模式开关切到 **Offboard**（`COM_FLTMODE<n> = 7`） | 此时还没解锁是正常的 |
| 3 | **遥控器手势解锁** | 一旦 armed，WAIT 的条件齐了，立刻自动起飞 |
| 4 | 任务自己跑完并上锁 | `DISARM` 是模块唯一主动发的动作 |

第 2 步能在"未解锁"时就切进 Offboard，是因为本模块从 `start` 起就 50 Hz 发
`offboard_control_mode` —— 这正是 §1 那个"必须先有信号才能切模式"的结论。
飞控上要先把 Offboard 分给模式开关：`COM_FLTMODE1..6`，值 `7`。

`COM_FLTMODE` 取值（`ManualControl::navStateFromParam`，本文件用到的几个加粗）：

| 值 | 模式 | 值 | 模式 | 值 | 模式 |
|---|---|---|---|---|---|
| 0 | Manual | 6 | Acro | 12 | Follow Target |
| 1 | Altitude | **7** | **Offboard** | 13 | PrecLand |
| **2** | **Position** | **8** | **Stabilized** | 14 | Orbit |
| 3 | Mission | 9 | Position Slow | 15 | VTOL Takeoff |
| **4** | **Hold（Loiter）** | 10 | Takeoff | 16 | Altitude Cruise |
| 5 | Return | 11 | Land | | |

> 反过来"先解锁、再切进 Offboard"也能跑（WAIT 的条件同样会齐），
> 但那样等于把已解锁的飞机切进 Offboard，切进去那一刻就起飞，
> 所以推荐先模式后解锁。

> ⚠ **这个顺序在 v1.17 上会被拒绝。** v1.17 的 `Commander::arm()` 里有：
>
> ```cpp
> if (_vehicle_control_mode.flag_control_manual_enabled) {
>     // 手动模式：只检查油门安全位置
> } else if (calling_reason == stick_gesture || rc_switch || rc_button) {
>     "Arming denied: switch to manual mode first"   → 拒绝
> }
> ```
>
> Offboard 永远是 `flag_control_manual_enabled == false`（`control_mode.cpp:111`），
> 所以 v1.17 上手势解锁**必须先在手动模式里做**，顺序要反过来：
> **手动模式（Stabilized/Altitude/Position）手势解锁 → 再切 Offboard**。
> v1.18 把那个 `else if` 整段删了，只剩下 `if (manual_enabled) { 油门安全检查 }`，
> 所以本仓库编译的 v1.18-beta 固件 / SITL 上可以按上表顺序来。

### 手势（ManualControl::processStickArming）

| 动作 | 摇杆 |
|---|---|
| 解锁 | 左杆油门最低 + 左杆偏航打满**右** + 右杆回中，**保持 1 s** |
| 退锁 | 左杆油门最低 + 左杆偏航打满**左** + 右杆回中，**保持 1 s** |

**要回中的只有右杆（roll/pitch）。左杆不要回中** —— 油门必须一直压在最低，
因为判定条件是 `input.throttle < -0.8f`（归一化后最低 = −1、中位 = 0、最高 = +1），
油门回到中位就是 0，手势判不出来：

```cpp
const bool right_stick_centered   = (fabsf(input.pitch) < 0.1f) && (fabsf(input.roll) < 0.1f);
const bool left_stick_lower_right = (input.throttle < -0.8f) && (input.yaw > 0.9f);
_stick_arm_hysteresis.set_state_and_update(left_stick_lower_right && right_stick_centered, ...);
```

油门自回中的发射机 / QGC 虚拟摇杆：**用手压到底按住**，松手弹回中位就等于没做完。
解锁之后油门放在哪都不参与控制（Offboard 里 `flag_control_manual_enabled == false`），
真正要松的是偏航杆 —— 那一下就是 `sticks_moving`，所以 `MAN_OVERRIDE_SPD = -1` 是必需的。

1 s 是 `manual_control` 里 `_stick_arm_hysteresis` 的固定迟滞。
飞行中途退锁 = 中止任务（`armed` 变 false → 安全兜底踢到 `DONE`）。

### 让手势能用的参数（⚠ 必须手动核对一次）

`param set-default` 只在参数还没被改过时才生效，**飞控上已经存过的值不会被覆盖**。
用 QGC 或 MAVLink Console 强制一次：

```
param show RC_MAP_ARM_SW MAN_ARM_GESTURE MAN_OVERRIDE_SPD MC_AIRMODE
param set  RC_MAP_ARM_SW 0
param set  MAN_ARM_GESTURE 1
param set  MAN_OVERRIDE_SPD -1
param save
```

| 参数 | 必须 | 不设会怎样 |
|---|---|---|
| `RC_MAP_ARM_SW` | `0` | 配了解锁开关的话，`ManualControl::updateParams` 启动时会把 `MAN_ARM_GESTURE` **自动改成 0**，手势直接没有 |
| `MAN_ARM_GESTURE` | `1` | 手势总开关，关了就只剩 GCS / 解锁开关 |
| `MAN_OVERRIDE_SPD` | `< 0` | 解锁后把偏航杆回中会被判成 `sticks_moving` → commander 切到 Position 模式 → 掉出 Offboard → 任务被安全兜底中止。（v1.17 固件上这个开关叫 `COM_RC_OVERRIDE`，默认 `1` = 只有 auto 模式可被摇杆接管、offboard 不接管，所以 v1.17 不用改） |
| `MC_AIRMODE` | `!= 2` | 偏航 airmode 会强制禁用手势（本机是 `1`，没问题） |

## 应急接管：从 Offboard 切回手动会不会"跳变"

### 水平 / 姿态 / 推力：不会

`FlightTaskManualPosition::activate()` 把 XY position setpoint 设成**当前位置**、XY velocity
setpoint 设成 0，Z 的 position setpoint 也设成当前高度、velocity setpoint 设成 0
（`FlightTaskManualAltitude::activate()`）。`FlightModeManager` 还会把上一个 task 的
`last_setpoint` 一起传进去。所以位置环这一拍的输出 ≈ 切之前那个"保持悬停"的输出，
**姿态和电机指令是连续的，没有阶跃**。

### 垂直：会立刻按油门杆走

`FlightTaskManualAltitude::_scaleSticks()` 每个周期都**无条件**用摇杆覆写
`_velocity_setpoint(2)`，而 `_updateAltitudeLock()` 里**没有任何"等杆回中"的闩锁**：

| 油门杆 | 结果 |
|---|---|
| 在中位，即 `\|throttle\| <= MAN_DEADZONE`（默认 0.1） | `apply_brake` 成立 → `_position_setpoint(2) = 当前位置` → **锁高度，不动** |
| 不在中位 | `_position_setpoint(2) = NAN` → 速度环直接用 `vel_max_down`（`MPC_Z_VEL_MAX_DN`，默认 1.5 m/s）→ **立刻下降** |

所以"跳变"不在姿态上，而在**高度指令**上：不是瞬间切推力，而是速度环用几百毫秒把下降速度
拉到 1.5 m/s —— 但在 1 m 高度上，那就是一秒内落地。

### ⚠ 手势解锁让这个坑必然发生

手势要求油门一直压在最低，解锁后油门就在最低。**所以解锁后直接拨到 Position/Altitude = 立刻下降。**

### 正确做法：两段式接管

| 步骤 | 动作 |
|---|---|
| 1 | 模式开关拨到 **Hold**（`COM_FLTMODE<n> = 4`）—— Hold 完全不吃摇杆，保持位置和高度 |
| 2 | 从容把**油门杆回中** |
| 3 | 再拨到 **Position**（值 `2`）手动接管 |

Hold 安全的原理和 Offboard 一样：`flag_control_manual_enabled = false`，摇杆不参与。
（v1.17 上还要确认 `COM_RC_OVERRIDE` 的 bit 0 关掉，否则动杆会从 Hold 被顶到 Position。）

### 模块会替你检测并告警

`my_mission` 现在检测油门是否回中，判据和 PX4 逐字一致（`|throttle| <= MAN_DEADZONE`）：

- 离开 Offboard 后 1 秒内，若当前模式是 Position / PositionSlow / Altitude / AltitudeCruise
  / Stabilized / Manual / Acro 而油门没回中 → 发 **Critical** 事件 + `PX4_ERR`：
  `Left Offboard with throttle stick off center: altitude will drop immediately`
- `my_mission status` 多打一行：
  `throttle 0.00 -> CENTERED (...) / OFF-CENTER (...)`

⚠️ **它只告警，拦不住下降** —— 模式是 commander 切的，模块既拦不住也不该去抢。
真正防住这件事的是上面的两段式接管。Hold / Return / Mission 这些不吃摇杆的模式不会告警。

## 怎么跑（SITL）

airframe `4022_gz_towed` 已经把 `towed_control start` 注释掉、改成自启 `my_mission start`
（原因见下面"冲突"），所以起来就是本模块在跑：

```
make px4_sitl gz_towed

# 1) 切 Offboard：模式开关，或者控制台
commander mode offboard

# 2) 解锁：SITL 没有真遥控器，用 QGC -> 应用设置 -> 虚拟摇杆 (Virtual Joystick)
#    它发 MAVLink MANUAL_CONTROL，手势判定用的是同一份 manual_control_setpoint，
#    然后做手势：油门最低 + 偏航打满右，保持 1 s。
#    不想用摇杆也可以直接：commander arm

# 观察
my_mission status
listener vehicle_status            # nav_state / arming_state
listener trajectory_setpoint
```

跑完之后：模块停在 `WAIT`，再切一次 Offboard + 解锁就会重跑一遍。
想彻底停掉：`my_mission stop`。

> 只用 SIH（不需要 Gazebo、秒级起停）时改成 `PX4_SIM_MODEL=sihsim_quadx`，
> 起来后手动 `my_mission start`（SIH 的 airframe 不会自启本模块）。

## 为什么位置目标不能一步跳过去

**这条也踩过坑。** 最早的 `FORWARD` 是这么写的：

```cpp
_target(0) += fwd_dist * cosf(psi);   // ← 给位置环一个 fwd_dist 米的阶跃
_target(1) += fwd_dist * sinf(psi);
```

现象：切进 `FORWARD` 的瞬间飞机猛地往前冲、姿态一下压到底。

原因全在 PX4 的位置环是**没有加速度限制的 P 环**（`PositionControl.cpp:130-140`）：

```
vel_sp = (_pos_sp - _pos) * MPC_XY_P          // 默认 0.95 m/s per m
vel_sp.xy() = constrainXY(..., MPC_XY_VEL_MAX) // 默认 12 m/s，只是天花板
acc_sp = (vel_sp - _vel) * MPC_XY_VEL_P_ACC    // 默认 1.8 m/s² per m/s
```

| `MYM_FWD_DIST` | 阶跃后 `vel_sp` | `acc_sp` | 倾角 `atan(a/g)` |
|---|---|---|---|
| 1 m | 0.95 m/s | 1.7 m/s² | 10° |
| 3 m | 2.85 m/s | 5.1 m/s² | 28° |
| 10 m | 9.5 m/s | 17.1 m/s² | 60° → 被 `MPC_TILTMAX_AIR`(45°) 削平 |
| ≥ 12.6 m | 顶到 12 m/s | — | 全程 45° |

⚠ **参数救不了这个**：
- `MPC_ACC_HOR` 只喂给 `GotoControl`（`MulticopterPositionControl.cpp:199`）；
  一旦外部在发 `trajectory_setpoint`，`GotoControl` 被整个跳过
  （同文件 `427-435`，`goto_setpoint_enable` 要求 `!_trajectory_setpoint_sub.updated()`）。
  所以 Offboard 的位置阶跃**没有任何加速度限制**。
- 调小 `MPC_XY_P` 确实能减小爆发速度，但它同时把悬停刚度、抗风、自动模式的位置响应
  一起拉低，是"全局副作用换局部效果"。

**正确做法：不要自己写参考生成器。** 现在 `FORWARD` 把目标点交给 PX4 自己的
`PositionSmoothing`（和 `GotoControl` 用的是同一套，`GotoControl.cpp:100`），
由它生成位置 / 速度 / **加速度** / jerk 四条曲线，然后整包发出去。

## 参考生成：直接复用 PX4 的 `PositionSmoothing`

### 为什么必须换掉"自己推斜坡"

自己推斜坡的做法是：`_target` 每拍朝终点挪 `v·dt`，`v` 由梯形曲线给出
（`MYM_FWD_SPD` 限速、`MYM_FWD_ACC` 限加减速），并把它作为 `velocity[0..1]`
前馈一起发。这个方案看起来对，实测**稳定过冲 0.39~0.45 m**
（gz_towed，`DIST=3.0`、`SPD=1.0`、`ACC=1.0`，7 次复现）。

原因在控制回路的结构里。`trajectory_setpoint.velocity` 在 PX4 里**不是速度指令**，
只是前馈；真正决定速度的是速度环

```
acc_sp = MPC_XY_VEL_P_ACC × (vel_sp − vel)      PositionControl.cpp:149-150
```

这是一阶惯性，时间常数 `τ = 1/MPC_XY_VEL_P_ACC` ≈ **0.556 s**
（增益量纲是 (m/s²)/(m/s) = 1/s，取倒数就是时间）。跟着一条**斜率恒定**的减速
曲线走时，稳态速度差恰好是 `a·τ`。日志实测这条差值从巡航的 0.10 m/s 收敛到
**0.60 m/s**，和 `1.0 × 0.556` 加上位置环那一份完全吻合。

于是在减速的那一秒里，飞机比参考**多飞 `τ × 巡航速度` ≈ 0.56 m**。
参考到达终点时，飞机已经在它前面 0.29 m、还剩 0.6 m/s；接着前馈归零，
位置环手里只有一个 0.29 m 的误差（`0.95 × 0.29 = 0.28 m/s`），
刹不住那 0.6 m/s，再冲 0.12 m 到峰值，然后像 ζ ≈ 0.69 的二阶系统那样荡回来。

两个关键结论：

- **多飞的距离是 `τ × 巡航速度`，和减速度 `a` 无关。** 把 `MYM_FWD_ACC` 调小只会让
  减速段变长，多飞的距离不变 —— **调参救不了**。
- **根子是"1 型回路跟踪抛物线参考"**。位置环只有一个积分（加速度→速度），
  对阶跃无差、对斜坡有差，对抛物线（恒减速）的稳态误差是 `a / (K_v · K_p)`。
  前馈只补了一阶（速度），补不到二阶（加速度）。

### 修法：把加速度前馈一起发出去

`trajectory_setpoint.acceleration` 在 `PositionControl.cpp:106` 被取值、
在 `:153` **直接**加进加速度指令，不经过速度环那个滞后。所以只要参考的加速度
一起发出去，`a·τ` 那一项从原理上就消失了。

`PositionSmoothing` 本来就产出加速度，而且它整套设计就是为此服务的：

| 它做了什么 | 解决什么 |
|---|---|
| 每拍先按上一拍解出的三段时长推进 dt，再以新状态为起点重解一次 | 不累积数值漂移，终点有保证 |
| 从飞机**当前的位置 / 速度 / 加速度**初始化（三阶状态） | 任何一个阶次上都没有跳变 |
| 速度上限由剩余距离反解，且**扣掉 jerk 建立减速度期间的滑行距离** | 刹车余量比 `sqrt(2ad)` 更保守 |
| 输出 position + velocity + **acceleration** + jerk | 前馈补到二阶 |
| 跟踪误差超限时按比例拉长积分时间 | 飞机落后时参考自己慢下来等它 |

### 限制怎么映射

原则：**能用 `MYM_` 参数的就用 `MYM_`，气动/机体限制用 PX4 的**，
口径和 `GotoControl.hpp:99-102` 完全一致。

| 平滑器的限制 | 来源 | 默认 |
|---|---|---|
| `setCruiseSpeed` | `MYM_FWD_SPD` | 本模块 |
| `setMaxAccelerationXY` | `MYM_FWD_ACC` | 本模块 |
| `setMaxVelocityXY` | `MPC_XY_VEL_MAX` | 12 m/s |
| `setMaxJerk` | `MPC_JERK_AUTO` | 4.0 m/s³ |
| `setMaxAllowedHorizontalError` | `MPC_XY_ERR_MAX` | 2.0 m |
| `setMaxAllowedVerticalError` | `MPC_Z_ERR_MAX` | 1.0 m |
| `setMaxVelocityZ` / `setMaxAccelerationZ` | `MPC_Z_V_AUTO_UP` / `MPC_ACC_UP_MAX` | 3.0 / 4.0 |

⚠ 这几个 PX4 参数都用 `param_find` 读（不进本模块参数表），和 `LNDMC_Z_VEL_MAX`
是同一个套路。**`setCruiseSpeed` 忘了设的症状是"速度永远上不去"**，
而加速度和 jerk 看起来都正常，很容易误判成控制问题。

### 到位判据

两件事都满足才算到：**轨迹**贴住终点（`MYM_ARRIVE`，默认 0.20 m），
**且飞机**速度 < 0.3 m/s。轨迹到了不等于飞机到了，中间还有跟踪误差。

位置口径比 PX4 任务航点的 `NAV_ACC_RAD`（10 m）严格得多 —— 任务航点是
"进圈算过、接着飞下一个"，我们要的是"停在这个点上"。

⚠ 进 `HOVER` 之前会把 `_target` **钉到真正的终点**。不钉的话，因为到位判据带
0.2 m 容差、而平滑器是渐近收敛的，飞机会永久停在离终点最多 0.2 m 的地方
（实测 0.14 m，比过冲还大）。

### 过冲 A/B：手写斜坡 vs `PositionSmoothing`

配置：gz_towed（`SYS_AUTOSTART=4022`）、`MYM_FWD_DIST=3.0`、`SPD=1.0`、`ACC=1.0`、
`MPC_JERK_AUTO=4.0`。每行一次完整任务，过冲 = 参考停下后 3 秒内飞机沿前进方向的
最大位移 − 目标距离（用 50 Hz 的 groundtruth 量）。

| | 停下时飞机速度 | 过冲 |
|---|---|---|
| **手写斜坡**（8 次） | 0.588 ~ 0.706 m/s | **+0.352 ~ +0.478 m** |
| **新方案**（4 次） | 0.255 ~ 0.311 m/s | **−0.028 ~ +0.069 m** |

- 均值从 **0.42 m 降到 0.016 m**（约 26 倍）
- **参考停下那一刻飞机的速度降了 2.2 倍**（0.65 → 0.27 m/s）—— 这正是
  `a·τ` 那一项被加速度前馈消掉的直接证据
- 新方案 4 次里有 2 次是**负过冲**（停在终点前一点点），正是我们想要的偏保守方向

复现方法：`build/px4_sitl_default/rootfs/log/` 下的 `.ulg`，用
`pyulog` 取 `trajectory_setpoint.position` 和 `vehicle_local_position_groundtruth`；
`trajectory_setpoint.acceleration` 全程非零是"前馈真的发出去了"的判据。

> ⚠ 量真值时要小心坐标原点：PX4 进程重启时 EKF 原点会重置到飞机当时的所在处，
> 而 gz 的 groundtruth 仍是世界坐标，两者差一个常量平移。所以真值必须用
> **它自己在起飞那一刻的位置**当原点，不能拿 setpoint 的起点去减。

### ⚠ 用 `dt` 的实测值

`PositionSmoothing` 靠 `dt` 推进轨迹（内部是 `_local_time += dt * stretch`），
`dt` 不准会让轨迹的"时间"和"几何"对不上。所以 `Run()` 里的 `dt` 用**实测时间差**，
并夹在 2~40 ms 之间（和 `MulticopterPositionControl.cpp:391-392` 一致）；
方式 0 的下降斜坡也用同一个 `dt` 积分。

### 这里踩过的坑（记下来，别再踩）

下面两条都是**已删除的那种手写斜坡**踩出来的，但教训仍然成立。

**坑一：不要用"投影"当跟踪误差口径。** 曾经用前进方向上的投影做限幅，理由是
"侧向偏差靠前进消不掉"。但 PX4 用的是 **2D 距离** —— 侧向偏差确实消不掉，
可它会被**方向判据**挡在外面一半，而且 2.0 m 的上限足够宽，不会卡死。
自定义口径只会让逻辑变复杂、还容易写错。

**坑二：限幅不能反馈进参考的状态量。** 曾经把限幅后的速度写回 `_ramp_speed`：

```
_ramp_speed{n+1} = (_ramp_speed{n} + a·dt) × s
```

稳态解是 `a·dt·s/(1-s)` —— `s=0.8` 时只有 **0.08 m/s**，斜坡永远起不来，
而且改 `MYM_FWD_SPD` 完全没用（上限项根本轮不到生效）。实测 33 秒只走了 2.65 m，
电池放完触发 failsafe。**正确顺序**：先算参考速度，**再**算限幅后的输出。


## ⚠ 冲突 1：`trajectory_setpoint` 不能在非 Offboard 时发

**这条踩过坑，是真 bug（已修）。**

`trajectory_setpoint` 是**共享** topic，非 Offboard 时它是
`FlightModeManager → mc_pos_control` 的通道，而且 **land detector 也在读它**：

```cpp
// MulticopterLandDetector.cpp:248
_in_descend = PX4_ISFINITE(tsp.velocity[2]) && (tsp.velocity[2] >= 1.1f * LNDMC_Z_VEL_MAX);
...
ground_contact &= _in_descend;      // 没被"指令下降"就永远不是 ground_contact
```

本模块的 setpoint 是 position-only（velocity = NaN）。如果在 WAIT / DONE 里也照发
（飞手手动飞的时候这两个状态一直存在），就会和飞行任务的有限速度 setpoint
**交替占坑**：land detector 每读到一份 NaN 就把 `_in_descend` 打成 false，
`ground_contact` 永远锁不上 → `maybe_landed` / `landed` 永远为 0
→ 于是 `Commander::disarm()` 的着地检查永远不过
→ **手势退锁、解锁开关、QGC 按钮、`COM_DISARM_LAND` 全部被拒**，
而且手势来源被拒时**不打任何日志**（`calling_reason != stick_gesture` 才打）。

实测证据（`log_0_2026-10-8-12-03-46.ulg`，POSCTL 落地后手势退锁失败）：

| 观察 | 值 |
|---|---|
| `trajectory_setpoint` 发布者 | **两个**，签名完全不同 |
| ↳ NaN velocity + 有限 position/yaw | 41 个样本，**全部是这个签名** → `my_mission` |
| ↳ 有限 velocity + NaN position/yaw | 123 个样本 → `FlightModeManager` 的飞行任务 |
| `offboard_control_mode{position=1}` | 全程在发（326 样本）→ `my_mission` 确实在跑 |
| `in_descend` | 逐拍 0/1 翻转 |
| `landed` | 起飞后到日志结束**全程为 0**（推力已低到 0.120） |
| `action_request` | t=24.036 **确实有一条 ACTION_DISARM / SOURCE_STICK_GESTURE** |

所以 `publishTrajectorySetpoint()` 现在**只在 `nav_state == OFFBOARD` 时发布**。
正确性：Offboard 下 `FlightModeManager::start_flight_task()` 没有 OFFBOARD 分支
→ `switchTask(FlightTaskIndex::None)` → `Run()` 里 `isAnyTaskActive()` 为假
→ 它不发布该 topic，`my_mission` 是唯一发布者；非 Offboard 时它独占，我们不碰。

这个 bug 还有第二个后果：`mc_pos_control` 也读同一条 topic，所以手动飞行时
位置控制器每 50 Hz 会被"只控位置、不管速度"的样本顶掉一拍，**手感和油门响应会发涩**。

## ⚠ 冲突 2：`offboard_control_mode` 是单实例 topic

`towed_control` 也在发它（`attitude=true`），两个同时跑会互抢，commander 只认最后写的那个。
所以 `4022_gz_towed` 里**只自启一个**：

- 现在：`my_mission start` 生效，`towed_control start` 被注释掉。
- 要跑拖曳测试：手动 `towed_control stop`（如果它没在跑则不用），再
  `my_mission stop; towed_control start`；或者改 airframe 里那两行的注释状态。

## 真机（CUAV Nora）

已经把这套接进 Nora 的板级配置：

| 改动 | 文件 |
|---|---|
| 开模块 | `boards/cuav/nora/default.px4board` 加 `CONFIG_MODULES_MY_MISSION=y` |
| 真机 airframe | `ROMFS/px4fmu_common/init.d/airframes/4021_towed_quad` |
| 注册 airframe | `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` |

编译 / 刷写：

```
make cuav_nora_default            # 编译
make cuav_nora_default upload     # 连上飞控刷写（或 QGC 里刷）
```

刷完在 QGC 把机架选成 **Towed Quad (4 lift + 2 forward thrust)**（`SYS_AUTOSTART = 4021`）。

### ⚠ 解锁前必须做的事

`4021_towed_quad` 里有三组**占位 / 假设值，不能直接飞**：

1. `CA_ROTOR0..5_*` 几何 —— 从 SITL 抄的，必须和实际机架核对
2. `CA_ROTOR0..5_CT` 单桨满推推力 [N] —— **必须台架实测**
3. `PWM_MAIN_FUNC1..6` 输出映射 —— 假设电调接在 PWM1..6 且顺序对应 `CA_ROTOR0..5`

> Nora 没有独立 IO 芯片，所以 `pwm_out` 的参数前缀是 **`PWM_MAIN`**，不是 `PWM_AUX`。

### 真机特有前提

- **需要位置源**（GPS / 光流 / VIO）：position 型 Offboard 没位置估计进不去
- `my_mission` 是 airframe 自启的，**切到 Offboard + 手势解锁就会自动起飞**
  → 第一次务必拆桨 / 系留
- `NAV_DLL_ACT` 若 `> 0`（`4021_towed_quad` 里是 `2`），commander 会把
  `gcs_connection_lost` 当成硬性条件 → **没连地面站就解不了锁**，
  报 `Preflight Fail: No connection to the GCS`。纯遥控手势起飞就把
  `NAV_DLL_ACT` 设成 `0`

## 前提

- EKF 位置有效（position 型 Offboard 要求 local position 有效，否则 arming check 会禁掉 Offboard）
- `COM_OF_LOSS_T` 默认 1.0 s，本模块 50 Hz 发布，不会超时
- `COM_DISARM_LAND` 默认 2.0 s（mode 1 时 PX4 自己也会上锁，和本模块的 disarm 谁先到都算数）
- 要把 Offboard 分配给模式开关：`COM_FLTMODE<n> = 7`
