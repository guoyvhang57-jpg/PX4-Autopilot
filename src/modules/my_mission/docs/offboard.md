# Offboard 位置控制（my_mission 的背景知识）

这份文档原来压在 `MyMission.cpp` 开头，占 144 行。搬出来之后代码只剩结构，
背景放这里。

---

## 1. 什么叫 "Offboard"

Offboard 不是一个具体的控制器，而是"**由外部源提供 setpoint**"的一种模式。
这个"外部源"可以是机载电脑，也可以是**飞控上的一个模块**（`my_mission` 就是），
所以做这件事**不需要机载电脑**。

commander 认不认这个模式，只看两件事：

**a)** `vehicle_status.nav_state == NAVIGATION_STATE_OFFBOARD`
飞手用模式开关（`COM_FLTMODE<n> = 7`）或 MAVLink 切进来。
本模块**不自己切模式**。

**b)** `offboard_control_mode` 这条 topic 有新数据：至少一个 flag 为 true，
且 `timestamp` 距今 < `COM_OF_LOSS_T`（默认 1.0 s）
→ `commander/HealthAndArmingChecks/checks/offboardCheck.cpp:49-54`

关键结论：**(b) 必须在切进 Offboard 之前就满足**，否则模式开关根本切不进去；
切进去之后也必须一直满足，断了就触发 offboard signal lost failsafe，
按 `COM_OBL_RC_ACT`（默认 0 = Position）把飞机切走。
这就是本模块从 `start` 起就 50 Hz 一直发 `offboard_control_mode` 的原因。

---

## 2. `offboard_control_mode` 决定"交给哪一级控制器"

PX4 不看你"想要什么姿态"，只看这条 topic 里哪个 flag 被置位
（`commander/ModeUtil/control_mode.cpp:111-129`）：

| flag | SetpointType | 效果 |
|---|---|---|
| `position` | Trajectory | allocation / rates / attitude / altitude / velocity / position / climb_rate 七组 flag 全开 → `mc_pos_control` 接管位置环，吃 `trajectory_setpoint` |
| `velocity` | Trajectory | 同上，但关掉 position（只控速度） |
| `acceleration` | Trajectory | 只要加速度，再关掉 position/altitude/climb_rate |
| `attitude` | Attitude | `mc_att_control` 直接吃，位置环闭嘴（`towed_control` 用的就是这个） |

本模块置 `position = true`。于是控制链是：

```
my_mission ──trajectory_setpoint──> mc_pos_control ──vehicle_attitude_setpoint──>
mc_att_control ──vehicle_rates_setpoint──> mc_rate_control ──torque/thrust──>
control_allocator ──actuator_motors──> 输出驱动（SITL: gz_bridge / 真机: pwm_out）
```

---

## 3. `trajectory_setpoint` 的约定（三个最容易写错的点）

**a) 坐标系是 NED**：x 北、y 东、**z 向下**。
所以"上升 1 m"是 `position[2] = -1.0`。符号写反，飞机会往地里扎。

**b) 单位是 m / m/s / m/s²**，都是**位置环的输入**，不是电机量、不是角度。

**c) NaN = 该通道不受控**（`TrajectorySetpoint.msg` 里明确写了）。
"给 0"是另一个意思：要求速度必须为 0。

`velocity[2]` 还有一个额外用途：**land detector 在读它**，见第 4 节。

---

## 4. position 型 Offboard 的额外前提

`offboardCheck.cpp:56` 要求 local position 有效（EKF 有位置估计），
否则这个模式会被 arming check 直接禁掉。所以本模块在位置无效时：

- 不发布位置（位置发 NaN）
- 绝不切进 TAKEOFF

### `velocity[2]` 和 land detector

`MulticopterLandDetector.cpp:248`：

```c
_in_descend = PX4_ISFINITE(tsp.velocity[2]) && (tsp.velocity[2] >= 1.1f * LNDMC_Z_VEL_MAX);
// 紧接着 :248 是  ground_contact &= _in_descend
```

所以：

- **自己下降时必须给有限的 `velocity[2]`**，否则 `_in_descend` 恒为 false，
  `ground_contact` 永远锁不上 → `maybe_landed` / `landed` 永远为 0 →
  之后发的 disarm 会被 commander 拒绝。本模块的 `MYM_DESC_RATE` 下限
  （1.1 × `LNDMC_Z_VEL_MAX` = 0.275 m/s）就是从这个判据来的。
- **非 Offboard 时不要发 `trajectory_setpoint`**。它是共享 topic，
  `FlightModeManager → mc_pos_control` 也在用；插进去的 NaN 速度会让
  land detector 的 `_in_descend` 逐拍翻转，落地检测彻底坏掉。
  实测日志 `log_0_2026-10-8-12-03-46.ulg`：`velocity[2]` 有 25% 的样本是 NaN，
  `in_descend` 逐拍 0/1 翻转，`landed` 全程为 0，所有上锁方式全部失效。

> 这条约束现在由 `MissionOutput::publishSetpoint()` 一处保证（内部判断
> `_offboard`），状态机不需要也不应该自己判断。

---

## 5. 状态机和飞手操作顺序

```
WAIT ──arm + 进 Offboard──> TAKEOFF ──到达 MYM_ALT──>
       ├─ MYR_ROPE_EN = 1 ──> TILT ──MYR_TILT_HOLD 秒──> HOVER
       ├─ MYM_FWD_DIST > 0 ──> FORWARD ──到位──────────> HOVER
       └─ 都没有 ────────────────────────────────────> HOVER
     ──MYM_HOVER 秒──> LAND ──> DISARM ──> DONE ──(已上锁)──> WAIT（可以再跑一遍）
```

各状态的细节见 `README.md` 的"状态机"一节。

本模块不自己 arm、也不自己切模式，只检测 `(armed && offboard && 位置有效)`。
所以飞手要按这个顺序来（顺序不能反，原因见第 1 节的关键结论）：

1. **等位置有效** —— position 型 Offboard 的硬前提（第 4 节）
2. **模式开关切到 Offboard 档**（`COM_FLTMODE<n> = 7`）
   此时还没解锁是正常的：本模块从 `start` 起就 50 Hz 发 `offboard_control_mode`，
   commander 因此允许在未解锁状态下就切进 Offboard。
3. **遥控器手势解锁**：左杆油门最低 + 偏航打满右 + 右杆回中，保持 1 s
   （退锁同位置、偏航打满左。1 s 是 `manual_control` 里
   `_stick_arm_hysteresis` 的固定迟滞；判定在 `ManualControl::processStickArming`）
   → 一旦 armed，WAIT 的三个条件齐了，立刻进 TAKEOFF 自动起飞。
4. 任务跑完模块自己上锁（DISARM 是唯一由本模块发起的动作），
   飞手把模式开关拨回去。飞行中途也可以用手势退锁来中止：
   armed 变 false 后安全兜底会把状态踢到 DONE。

手势解锁能成立的前提（airframe 里已写成默认值，但飞控上已经存过的参数
不会被 `param set-default` 覆盖，必须手动核对一次）：

| 参数 | 值 | 不满足会怎样 |
|---|---|---|
| `RC_MAP_ARM_SW` | 0 | 配了解锁开关的话，`manual_control::updateParams` 启动时会把 `MAN_ARM_GESTURE` 自动改成 0，手势直接没有 |
| `MAN_ARM_GESTURE` | 1 | 手势总开关 |
| `MAN_OVERRIDE_SPD` | < 0 | 否则解锁后把偏航杆回中会被判成 `sticks_moving`，commander 立刻切到 Position 模式 → 掉出 Offboard → 本模块中止 |
| `MC_AIRMODE` | != 2 | 偏航 airmode 也会强制禁用手势 |

> v1.17 固件上对应的开关是 `COM_RC_OVERRIDE`（默认 1 = 只有 auto 模式可被摇杆
> 接管、offboard 不接管），所以 v1.17 上不用改 `MAN_OVERRIDE_SPD`。

---

## 6. 和 `towed_control` 的冲突（重要）

`offboard_control_mode` 是**单实例** topic。`towed_control` 也在发它
（`attitude = true`）。两个模块同时跑 = 互相覆盖，commander 只认
"最后写进去"的那一份。所以 airframe 里只自启一个；要切换必须先 `stop`
掉另一个。
