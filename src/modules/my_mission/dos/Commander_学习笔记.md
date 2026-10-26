# Commander 学习笔记

> **来源**：从 `src/modules/commander/` 源码里提取的中文注释。
> 这些注释原本直接写在上游源码中，每次同步上游都会冲突，所以抽出来单独存。
> 源码已恢复为上游原样。

> - 无损备份补丁：`Commander_学习笔记.patch`（同目录）（含完整 diff，可完整还原回源码）
> - 提取时间：2026-10-14 · 基线：PX4 `release/1.18`

---

## `src/modules/commander/Commander.cpp`

### `transition_result_t Commander::disarm(arm_disarm_reason_t calling_reason, bool f`

*源码位置：约第 752 行*

- 强制检查，默认false，想跳过显式写true
- 在landed，maybe_land，或一些没有明确降落的机型
- 在旋翼模式，_vehicle_control_mode中的flag_control_manual_enabled（手动控制为真），没有爬升率控制
- 看disarm理由是不是遥控器指令。手势/拨杆，按钮
- 拒绝条件，还有是否要报警
- 决定上锁，清一些标志位

### `void Commander::run()`

*源码位置：约第 2018 行*

- 把vtol_vehicle_status话题中数据copy到_vtol_vehicle_status（command成员），主要是landed被使用，_home_position对象往home_position话题中发布home点信息
- 抛飞，不用管
- 飞行器当前处于自动任务模式，并且任务结果消息显示任务尚未完成为真
- _home_position在landDetectorUpdate定好home点后根据实际情况维护home点（只往_home_position成员内部写）
- 自动上锁，根据自动上锁参数，是否在任务中，外部模式，kill开关，_vehicle_land_detected话题数据，在降落后，解锁未操作，kill开关后，执行disarm函数上锁
- 不改变什么成员变量，只是发送disarm动作
- 功能 A：任务有效性变化 → 响一声
- 已解锁
- 不在落地状态（!landed）
- mission_result.timestamp >= nav_state_timestamp —— 结果比当前模式更新（防止旧结果把刚切进去的新模式踢掉）
- mission_result.finished（任务完成或被清空）
- 没有外部 mode executor 负责（modeExecutorInCharge() == AUTOPILOT_EXECUTOR_ID）——外部执行器自己会管，别抢
- 然后分两种：
- 当前在 AUTO_TAKEOFF / AUTO_VTOL_TAKEOFF → 强制切 Hold（force=true，来源 ModeExecutor）——起飞完成后的正常去处
- 当前在 AUTO_MISSION → 切 Hold，但不强制（要过 canRun），且只在"意图仍是 AUTO_MISSION"时才切（避免覆盖已经排队的 RTL 等请求）
- ，属于上层意图，要过_user_mode_intention成员的change检测
- 在解锁模式下检测摇杆动作，如果比设定值大，经过条件判断通过user_mode写入意图
- 遥控信号丢失处理
- 丢失后给_status_changed = true
- 通过订阅无人机数据，判断是否姿态出问题，高度异常下降，螺旋桨出问题带来振动异常，然后写入_failure_detector对象内部flag
- 外部模式管理，目前不看

### `bool Commander::handleModeIntentionAndFailsafe()`

*源码位置：约第 2543 行*

- 立刻知道"意图是否被受理"（true/false），以及"本拍有过一次请求"（_had_mode_change，读走即清）

### `void Commander::manualControlCheck()`

*源码位置：约第 3161 行*

- 摇杆运动超过某一值&&四旋翼模式&&可以被接管的模式（mossion，offboard）
- 想要改变模式，一般都要先通过_user_mode_intentio登记意图，然后再由failsafe裁决
- 给出操作成功信息（灯或蜂鸣器以及mavlink消息）

## `src/modules/commander/failsafe/framework.cpp`

### `uint8_t FailsafeBase::update(const hrt_abstime &time_us, const State &state, boo`

*源码位置：约第 57 行*

- 在解锁和上锁时，把状态归位但保留还为解决的问题，主要操作ActionOptions _actions[8]
- 看两种情况是否发生了模式变化

## `src/modules/commander/failure_detector/FailureDetector.cpp`

### `bool FailureDetector::update(const vehicle_status_s &vehicle_status, const vehic`

*源码位置：约第 57 行*

- 读取姿态数据，和阈值对比，要持续超过，判断是否出错，写flags.roll / flags.pitch
- 判断"是不是在不受控地掉高度" 写 flags.alt，falg是_failure_detector_status对象成员
- 参数_param_fd_ext_ats_en是是否要根据外部数据判断pwm信号处问题了吗
- 通过加速度计的水平与垂直振动标准差差异来判断某个桨叶坏了、缺角、或电机异常
