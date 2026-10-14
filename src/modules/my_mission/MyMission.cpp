/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 ****************************************************************************/

/**
 * @file MyMission.cpp
 *
 * Offboard 位置控制的最小任务示例。下面按"是什么 → 为什么 → 怎么走"把 PX4 的
 * offboard 讲清楚。
 *
 * ===========================================================================
 * 1. 什么叫 "Offboard"
 * ===========================================================================
 * Offboard 不是一个具体的控制器，而是"**由外部源提供 setpoint**"的一种模式。
 * 这个"外部源"可以是机载电脑，也可以是**飞控上的一个模块**（本模块就是），
 * 所以做这件事**不需要机载电脑**。
 *
 * commander 认不认这个模式，只看两件事：
 *
 *   a) vehicle_status.nav_state == NAVIGATION_STATE_OFFBOARD
 *      飞手用模式开关（COM_FLTMODE<n> = 7）或 MAVLink 切进来。
 *      本模块**不自己切模式**。
 *
 *   b) offboard_control_mode 这条 topic 有新数据：至少一个 flag 为 true，
 *      且 timestamp 距今 < COM_OF_LOSS_T（默认 1.0 s）
 *      → commander/HealthAndArmingChecks/checks/offboardCheck.cpp:49-54
 *
 * 关键结论：(b) 必须在切进 Offboard **之前**就满足，否则模式开关根本切不进去；
 * 切进去之后也必须一直满足，断了就触发 offboard signal lost failsafe，
 * 按 COM_OBL_RC_ACT（默认 0 = Position）把飞机切走。
 * 这就是本模块从 start 起就 50 Hz 一直发 offboard_control_mode 的原因。
 *
 * ===========================================================================
 * 2. offboard_control_mode 决定"交给哪一级控制器"
 * ===========================================================================
 * PX4 不看你"想要什么姿态"，只看这条 topic 里哪个 flag 被置位
 * （commander/ModeUtil/control_mode.cpp:111-129）：
 *
 *   position      = true → SetpointType::Trajectory
 *                          allocation / rates / attitude / altitude / velocity /
 *                          position / climb_rate 七组 flag 全开
 *                          → mc_pos_control 接管位置环，吃 trajectory_setpoint
 *   velocity      = true → 同上，但关掉 position（只控速度）
 *   acceleration  = true → 只要加速度，再关掉 position/altitude/climb_rate
 *   attitude      = true → SetpointType::Attitude，mc_att_control 直接吃，
 *                          位置环闭嘴（towed_control 用的就是这个）
 *
 * 本模块置 position = true。于是控制链是：
 *
 *   本模块 ──trajectory_setpoint──> mc_pos_control ──vehicle_attitude_setpoint──>
 *   mc_att_control ──vehicle_rates_setpoint──> mc_rate_control ──torque/thrust──>
 *   control_allocator ──actuator_motors──> 输出驱动（SITL: gz_bridge / 真机: pwm_out）
 *
 * ===========================================================================
 * 3. trajectory_setpoint 的约定（三个最容易写错的点）
 * ===========================================================================
 *   a) 坐标系是 **NED**：x 北、y 东、**z 向下**。
 *      所以"上升 1 m"是 position[2] = -1.0。符号写反，飞机会往地里扎。
 *   b) 单位是 m / m/s / m/s²，都是**位置环的输入**，不是电机量、不是角度。
 *   c) **NaN = 该通道不受控**（TrajectorySetpoint.msg 里明确写了）。
 *      本模块只写 position，velocity / acceleration / jerk 全给 NaN，
 *      让位置环自己规划速度。"给 0"是另一个意思：要求速度必须为 0。
 *
 * ===========================================================================
 * 4. position 型 Offboard 的额外前提
 * ===========================================================================
 * offboardCheck.cpp:56 要求 local position 有效（EKF 有位置估计），
 * 否则这个模式会被 arming check 直接禁掉。所以本模块在位置无效时：
 *   - 不发布位置（位置发 NaN，见 publishTrajectorySetpoint）
 *   - 绝不切进 TAKEOFF
 *
 * ===========================================================================
 * 5. 状态机 + 飞手操作顺序
 * ===========================================================================
 *   WAIT ──arm + 进 Offboard──> TAKEOFF ──到 MYM_ALT──> [FORWARD ──到前方点──>] HOVER
 *        ──MYM_HOVER 秒──> LAND ──> DISARM ──> DONE ──(已上锁)──> WAIT（可以再跑一遍）
 *
 *   FORWARD 只在 MYM_FWD_DIST > 0 时插入（= 0 时行为跟以前完全一样）。
 *   目标点 = **起飞点** 沿"起飞那一刻锁定的机头方向"前方 MYM_FWD_DIST 米，
 *   MYM_FWD_DIR 再叠一个相对偏角（0 = 正前方，+90 = 右侧）。见 Run() 的 TAKEOFF 分支。
 *
 *   ⚠ "正前方"完全由**航向估计**决定（GPS 双天线 / 磁罗盘 / EKF）。航向歪多少度，
 *     飞出去的方向就歪多少度，而且 MYM_FWD_DIST 越大偏得越远（3 m 上偏 30° = 1.5 m）。
 *     所以第一次调这个参数时：MYM_FWD_DIST 先给小（1~2 m），MYM_ALT 给高（≥2 m），
 *     在开阔场地、飞机朝向你、随时能切回 Position 的状态下试。
 *
 * 本模块不自己 arm、也不自己切模式，只检测 (armed && offboard && 位置有效)。
 * 所以飞手要按这个顺序来（顺序不能反，原因见 §1 的关键结论）：
 *
 *   1) 等位置有效 —— position 型 Offboard 的硬前提（§4）
 *   2) 模式开关切到 Offboard 档（COM_FLTMODE<n> = 7）
 *      此时还没解锁是正常的：本模块从 start 起就 50 Hz 发 offboard_control_mode，
 *      commander 因此允许在未解锁状态下就切进 Offboard。
 *   3) **遥控器手势解锁**：左杆油门最低 + 偏航打满右 + 右杆回中，保持 1 s
 *      （退锁同位置、偏航打满左。1 s 是 manual_control 里 _stick_arm_hysteresis
 *        的固定迟滞；判定在 ManualControl::processStickArming）
 *      → 一旦 armed，WAIT 的三个条件齐了，立刻进 TAKEOFF 自动起飞。
 *   4) 任务跑完模块自己上锁（DISARM 是唯一由本模块发起的动作），
 *      飞手把模式开关拨回去。飞行中途也可以用手势退锁来中止：
 *      armed 变 false 后 §2 的安全兜底会把状态踢到 DONE。
 *
 * 手势解锁能成立的前提（airframe 里已写成默认值，但飞控上已经存过的参数
 * 不会被 param set-default 覆盖，必须手动核对一次）：
 *
 *   RC_MAP_ARM_SW = 0       否则 manual_control::updateParams 启动时会把
 *                           MAN_ARM_GESTURE 自动改成 0，手势直接没有
 *   MAN_ARM_GESTURE = 1     手势总开关
 *   MAN_OVERRIDE_SPD < 0    否则解锁后把偏航杆回中会被判成 sticks_moving，
 *                           commander 立刻切到 Position 模式 -> 掉出 Offboard
 *                           -> 本模块的安全兜底把任务中止。
 *                           （v1.17 固件上对应的开关是 COM_RC_OVERRIDE，
 *                             默认 1 = 只有 auto 模式可被摇杆接管、offboard
 *                             不接管，所以 v1.17 上不用改这一项。）
 *   MC_AIRMODE != 2         偏航 airmode 也会强制禁用手势
 *
 * ===========================================================================
 * 6. 和 towed_control 的冲突（重要）
 * ===========================================================================
 * offboard_control_mode 是**单实例** topic。towed_control 也在发它（attitude=true）。
 * 两个模块同时跑 = 互相覆盖，commander 只认"最后写进去"的那一份。
 * 所以 airframe 里只自启一个；要切换必须先 stop 掉另一个。
 */

#include "MyMission.hpp"

#include <mathlib/mathlib.h>

#include <px4_platform_common/events.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

ModuleBase::Descriptor MyMission::desc{task_spawn, custom_command, print_usage};

MyMission::MyMission()
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

MyMission::~MyMission()
{
}

bool MyMission::init()
{
	// 50 Hz。这个数怎么来的：
	//   下限由 COM_OF_LOSS_T 决定（默认要 1 s 内至少一份新数据，2 Hz 就够）；
	//   取 50 Hz 是为了两点：① 和 mc_pos_control 处于同一个量级的更新率，
	//   位置 setpoint 不会成为瓶颈；② 留足余量 —— CPU 抖动、日志写入、
	//   工作队列被别的模块占用时，1 s 的窗口其实很容易踩线。
	ScheduleOnInterval(20_ms);

	// 接管安全的油门中位判据跟着 PX4 走：具体阈值来自 MAN_DEADZONE（默认 0.1）
	updateExternalParams();

	return true;
}

const char *MyMission::stateName(State state)
{
	switch (state) {
	case State::Wait:    return "WAIT";

	case State::Takeoff: return "TAKEOFF";

	case State::Forward: return "FORWARD";

	case State::Hover:   return "HOVER";

	case State::Land:    return "LAND";

	case State::Disarm:  return "DISARM";

	case State::Done:    return "DONE";
	}

	return "?";
}

bool MyMission::localPositionValid(const vehicle_local_position_s &pos, hrt_abstime now) const
{
	// xy_valid / z_valid 是 EKF 给的"这两个通道现在能不能信"。
	// 再加一个 500 ms 的新鲜度判断：EKF 挂了之后 topic 还在，
	// 只看 valid 位的话会拿旧数据继续飞。
	return pos.xy_valid && pos.z_valid && (now < pos.timestamp + 500_ms);
}

bool MyMission::reachedTarget(const vehicle_local_position_s &pos) const
{
	if (!_target_valid) {
		return false;
	}

	const matrix::Vector3f p(pos.x, pos.y, pos.z);
	const matrix::Vector3f v(pos.vx, pos.vy, pos.vz);

	// 到位 = 位置误差小 **且** 速度也小。
	// 只判位置的话，飞过目标点的那一瞬间误差也是 0，会被误判成"到了"。
	return (p - _target).norm() < _param_mym_arrive.get() && v.norm() < 0.3f;
}

void MyMission::holdCurrentPosition(const vehicle_local_position_s &pos, const vehicle_attitude_s &att, bool pos_valid)
{
	// 把目标钉在"当前位置"。WAIT / DONE 阶段这样发，飞机不会动；
	// 万一这时飞手切进 Offboard，位置环的目标就是当前位置，不会突然冲出去。
	if (pos_valid) {
		_target(0) = pos.x;
		_target(1) = pos.y;
		_target(2) = pos.z;
		_target_valid = true;
	}

	// 航向取当前姿态。如果不给 yaw（或给 0），setpoint 里的航向和实际航向
	// 可能差 180°，位置环会先原地转半圈再飞。
	_yaw_sp = matrix::Eulerf(matrix::Quatf(att.q)).psi();
}

void MyMission::publishOffboardControlMode(hrt_abstime now)
{
	offboard_control_mode_s ocm{};

	// timestamp 必须**每拍刷新**：commander 就是靠
	// "now - ocm.timestamp < COM_OF_LOSS_T" 判断外部信号还在不在。
	// 只发一次、或者忘了刷时间戳，都会被判成 offboard signal lost。
	ocm.timestamp = now;

	// 唯一要置的 flag。它同时决定两件事：
	//   1) commander 认为 "offboard available" —— 只要**任意一个** flag 为 true 就够
	//   2) 进去之后 mc_pos_control 接管位置环 —— control_mode.cpp:114
	//
	// 其余 velocity / acceleration / attitude / body_rate / thrust_and_torque /
	// direct_actuator 必须保持 false：commander 是**按顺序**取第一个为 true 的分支
	// （control_mode.cpp:111-129），同时置多个会选中你没想到的那个。
	ocm.position = true;
	_ocm_pub.publish(ocm);
}

void MyMission::publishTrajectorySetpoint(hrt_abstime now, float z_velocity)
{
	// ===================================================================
	// 只在真正处于 Offboard 时才发 —— 这不是优化，是正确性要求
	// ===================================================================
	// trajectory_setpoint 是**共享** topic，非 Offboard 时它是
	//   FlightModeManager → mc_pos_control
	// 的通道，而且 **land detector 也在读它**来判断"是否被指令下降"
	//   MulticopterLandDetector.cpp:248
	//     _in_descend = PX4_ISFINITE(tsp.velocity[2]) && (velocity[2] >= 1.1 * LNDMC_Z_VEL_MAX)
	//   随后 ground_contact &= _in_descend
	//
	// 本模块的 setpoint 是 position-only、velocity = NaN。如果在 WAIT/DONE 里
	// 也照发（这两状态在飞手手动飞的时候一直存在），就会和飞行任务的有限速度
	// setpoint 交替占坑：land detector 每读到一份 NaN 就把 _in_descend 打成 false，
	// ground_contact 永远锁不上 → maybe_landed/landed 永远为 0
	// → 所有上锁方式（手势、解锁开关、QGC 按钮、COM_DISARM_LAND）全部被拒。
	//
	// 实测 ulg log_0_2026-10-8-12-03-46.ulg（POSCTL 手动落地后手势退锁失败）：
	//   trajectory_setpoint.velocity[2] 有 25% 的样本是 NaN，与任务的 0.700 交替；
	//   vehicle_land_detected.in_descend 逐拍 0/1 翻转；landed 全程为 0。
	// 而 FlightTaskManualPosition/ManualAltitude 从不把 velocity 置 NaN，
	// 所以那些 NaN 只可能来自本模块。
	if (!_in_offboard) {
		return;
	}

	trajectory_setpoint_s tsp{};
	tsp.timestamp = now;

	// ---- 位置（NED，向上为负）----
	if (_target_valid) {
		tsp.position[0] = _target(0);
		tsp.position[1] = _target(1);
		tsp.position[2] = _target(2);

	} else {
		// 还没有有效位置估计 → 位置发 NaN（= 不控制位置）。
		// 千万不要发 (0,0,0)：那会被理解成"飞到 EKF 原点"。
		tsp.position[0] = NAN;
		tsp.position[1] = NAN;
		tsp.position[2] = NAN;
	}

	// ---- 其余通道全部"不控制" ----
	// NaN 的语义是"这个状态不要你管"，位置环会自己规划出可行的速度/加速度。
	// 只写 position、其余全 NaN，是最常见也最安全的 offboard 位置用法。
	for (int i = 0; i < 3; i++) {
		tsp.velocity[i] = NAN;
		tsp.acceleration[i] = NAN;
		tsp.jerk[i] = NAN;
	}

	// ---- 可选的 z 速度前馈 ----
	// PositionControl 的结构是 P-位置环 + 速度前馈（两者相加，不是二选一）：
	//   PositionControl::_positionControl():
	//       vel_sp_position = (_pos_sp - _pos) * gain_P;
	//       addIfNotNanVector3f(_vel_sp, vel_sp_position);     // NaN 的项不参与
	// 所以给 velocity[2] 不会和 position[2] 打架。
	//
	// 自己下降时**必须**给：land detector 判断"是否被指令下降"读的就是它
	//   MulticopterLandDetector.cpp:248
	//       _in_descend = PX4_ISFINITE(tsp.velocity[2]) && (velocity[2] >= 1.1 * LNDMC_Z_VEL_MAX);
	// 只发 position（velocity = NaN）会让 _in_descend 恒为 false，
	// ground_contact 永远锁不上 -> landed 永远为 0 -> 后面发的 disarm 被 commander 拒。
	if (PX4_ISFINITE(z_velocity)) {
		tsp.velocity[2] = z_velocity;
	}

	// yaw 每拍都给锁存值（不然可能自己转）；yawspeed 给 NaN = 不控制偏航角速度。
	tsp.yaw = _yaw_sp;
	tsp.yawspeed = NAN;

	_traj_sp_pub.publish(tsp);
}

void MyMission::publishVehicleCommand(uint16_t command, float param1, float param2)
{
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = command;
	cmd.param1 = param1;
	cmd.param2 = param2;

	// commander 的校验（Commander.cpp:862）：
	//   target_system == 自己的 sysid **或 0（广播）** 才处理。
	// 本模块和 commander 在同一个进程里，用广播 0 最省事，不用去查自己的 sysid。
	cmd.target_system = 0;
	cmd.target_component = 0;

	// 填上来源，方便 commander / 日志区分是谁发的
	cmd.source_system = 1;
	cmd.source_component = 1;

	// false = 内部命令。commander 有些分支会区分内外来源（比如 arm/disarm 的
	// arm_disarm_reason），外部来的命令校验更严。
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);
}

// ===========================================================================
// 接管安全：油门杆回中检测
// ===========================================================================
// 为什么需要它：飞手把模式开关拨出 Offboard 的那一刻，高度通道立刻由油门杆接管。
// PX4 在 FlightTaskManualAltitude 里的行为是（每拍都跑，没有"等杆回中"的闩锁）：
//
//   _scaleSticks():
//       vel_max_z = (throttle < 0) ? MPC_Z_VEL_MAX_DN : MPC_Z_VEL_MAX_UP;
//       _velocity_setpoint(2) = vel_max_z * -throttle_expo;      // NED，正 = 下降
//
//   _updateAltitudeLock():
//       apply_brake = fabsf(throttle_expo) <= FLT_EPSILON;
//       杆在中位 -> _position_setpoint(2) = 当前位置（锁高度）
//       杆不在中位 -> _position_setpoint(2) = NAN（速度环直接用上面那个速度）
//
// 所以"油门在最低"进 Position/Altitude = 立刻以 MPC_Z_VEL_MAX_DN（默认 1.5 m/s）下降。
// 而手势解锁结束后油门正好就在最低 —— 这个坑是真实存在的。
//
// 本模块只做"检测 + 告警"：模式是 commander 切的，模块拦不住，也不该去抢。
void MyMission::updateExternalParams()
{
	// MAN_DEADZONE：判"油门算不算在中位"，和
	// FlightTaskManualAltitude::_updateAltitudeLock() 的 apply_brake 对齐
	if (_param_man_deadzone == PARAM_INVALID) {
		_param_man_deadzone = param_find("MAN_DEADZONE");
	}

	float deadzone = _man_deadzone;

	if ((_param_man_deadzone != PARAM_INVALID) && (param_get(_param_man_deadzone, &deadzone) == PX4_OK)) {
		_man_deadzone = deadzone;
	}

	// LNDMC_Z_VEL_MAX：land detector 认"被指令下降"的门槛
	//   _in_descend 要求 trajectory_setpoint.velocity[2] >= 1.1 * LNDMC_Z_VEL_MAX
	// 自己下降时（MYM_LAND_MODE=0）必须发够这个速度，否则 landed 永远上不去。
	if (_param_lndmc_z_vel_max == PARAM_INVALID) {
		_param_lndmc_z_vel_max = param_find("LNDMC_Z_VEL_MAX");
	}

	float lndmc_z_vel_max = _in_descend_min_vel / 1.1f;

	if ((_param_lndmc_z_vel_max != PARAM_INVALID) && (param_get(_param_lndmc_z_vel_max, &lndmc_z_vel_max) == PX4_OK)) {
		_in_descend_min_vel = 1.1f * lndmc_z_vel_max;
	}
}

bool MyMission::throttleStickCentered(const manual_control_setpoint_s &manual_control) const
{
	// 和 PX4 判"油门在中位"逐字对齐：
	//   FlightTaskManualAltitude::_updateAltitudeLock()
	//     apply_brake = fabsf(_sticks.getThrottleZeroCenteredExpo()) <= FLT_EPSILON
	//   Sticks::getThrottleZeroCenteredExpo() = expo(deadzone(throttle, MAN_DEADZONE), 0.6)
	//   math::deadzone() 在 |x| <= deadzone 时返回 0，而 expo(0) = 0
	//   => "在中位" 等价于 |throttle| <= MAN_DEADZONE（默认 0.1，归一化行程）
	return manual_control.valid && (fabsf(manual_control.throttle) <= _man_deadzone);
}

void MyMission::checkTakeoverSafety(const vehicle_status_s &status, const manual_control_setpoint_s &manual_control)
{
	const int nav = status.nav_state;

	// 高度直接由油门杆决定的模式（杆在中位 = 锁高度，否则 = 升降速度指令）
	const bool altitude_from_throttle = (nav == vehicle_status_s::NAVIGATION_STATE_POSCTL)
					    || (nav == vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW)
					    || (nav == vehicle_status_s::NAVIGATION_STATE_ALTCTL)
					    || (nav == vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE);

	// 手动推力模式：更狠 —— 杆在最低就是零推力，直接掉高度
	const bool manual_thrust = (nav == vehicle_status_s::NAVIGATION_STATE_STAB)
				   || (nav == vehicle_status_s::NAVIGATION_STATE_MANUAL)
				   || (nav == vehicle_status_s::NAVIGATION_STATE_ACRO);

	// Hold / Return / Mission 这些不吃摇杆的模式是安全的，不告警
	if (!altitude_from_throttle && !manual_thrust) {
		return;
	}

	if (throttleStickCentered(manual_control)) {
		return;
	}

	_takeover_warned = true;

	/* EVENT
	 * @description
	 * The vehicle left Offboard mode while the throttle stick was not centered.
	 * In Position/Altitude the altitude is then commanded by the stick
	 * (descend at MPC_Z_VEL_MAX_DN), in Stabilized/Manual there is no thrust at all.
	 * Center the throttle stick first (or switch to Hold) before taking over.
	 */
	events::send(events::ID("my_mission_takeover_throttle_off_center"),
	{events::Log::Critical, events::LogInternal::Info},
	"Left Offboard with throttle stick off center: altitude will drop immediately");

	PX4_ERR("my_mission: left Offboard with throttle off center (%.2f, |x| > %.2f): altitude will drop now",
		(double)manual_control.throttle, (double)_man_deadzone);
}

void MyMission::Run()
{
	// 【模块退出协议】外部执行 `my_mission stop` 时 should_exit() 变 true。
	// 必须先 ScheduleClear() 再 exit_and_cleanup()，否则工作队列可能在对象已经
	// 释放之后还调用一次 Run()。
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = 0.02f;   // 必须和 init() 里的 20_ms 一致：下降斜坡靠它积分

	// 参数在线更新：QGC 或命令行改完 MYM_* 立即生效，不用重启模块、更不用重编。
	if (_parameter_update_sub.updated()) {
		updateParams();
		updateExternalParams();   // MAN_DEADZONE / LNDMC_Z_VEL_MAX 一起刷新
	}

	// ---- 读状态 ----
	// copy() = 取最新一份（不判断有没有更新）；update() = 有新数据才覆盖。
	// 这里只关心"当前是什么状态"，所以 copy 就够；local_position 用 update 是为了
	// 少一次无谓拷贝（两者对逻辑没有区别）。
	vehicle_status_s status{};
	actuator_armed_s armed{};
	vehicle_local_position_s pos{};
	vehicle_attitude_s att{};
	vehicle_land_detected_s land{};
	manual_control_setpoint_s manual_control{};

	_vehicle_status_sub.copy(&status);
	_armed_sub.copy(&armed);
	_local_pos_sub.update(&pos);
	_attitude_sub.copy(&att);
	_land_detected_sub.copy(&land);
	_manual_control_sub.copy(&manual_control);

	const bool pos_valid = localPositionValid(pos, now);
	const bool offboard = (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	// publishTrajectorySetpoint() 用它来决定"这一拍该不该发 setpoint"，
	// 不能乱发的原因见那个函数开头（会和 land detector 抢同一条 topic）。
	_in_offboard = offboard;

	// ---- 1) 先发 offboard_control_mode ----
	// 放在状态机之前、而且**任何状态都发**，三个原因：
	//   a) 切进 Offboard 之前就得让 commander 看到信号，否则模式开关切不进去；
	//   b) 进去之后不能断，断了就是 offboard loss failsafe；
	//   c) WAIT / DONE 也可能正处在 Offboard 里（比如刚上锁还没切走），一样要续命。
	publishOffboardControlMode(now);

	// ---- 2) 安全兜底 ----
	// 只有"正在起飞 / 前飞 / 悬停"时，掉出 Offboard 或上锁才算异常，直接中止。
	// LAND 阶段允许离开 Offboard —— 方式 1 的 AUTO_LAND 本来就会把 nav_state 切走；
	// DISARM / DONE 本来就已经落地或结束，不算异常。
	if ((_state == State::Takeoff || _state == State::Forward || _state == State::Hover)
	    && (!offboard || !armed.armed)) {
		PX4_WARN("my_mission: abort (offboard=%d armed=%d)", (int)offboard, (int)armed.armed);
		_state = State::Done;
		_state_start = now;   // Done 的计时从这里开始（2b 的接管检查用它开 1 秒窗口）
	}

	// ---- 2b) 接管安全：离开 Offboard 后油门没回中 ----
	// 放在 Done 的头 1 秒内检查，而不是上面那个 if 里：nav_state 的更新比
	// mode 开关晚 1~2 拍，在 Done 里再看几拍容错更大。
	// armed 才检查 —— 如果是被 kill/termination 上锁退出的，不会再有掉高度的问题。
	if (!_takeover_warned && (_state == State::Done) && armed.armed && (elapsedSince(_state_start, now) < 1.f)) {
		checkTakeoverSafety(status, manual_control);
	}

	switch (_state) {

	case State::Wait:
		// 进 Offboard 之前：目标 = 当前位置。位置环就算已经在跑也不会动。
		holdCurrentPosition(pos, att, pos_valid);
		publishTrajectorySetpoint(now);

		// 三个条件齐了才起飞：
		//   armed     —— 电机允许转
		//   offboard  —— 飞手已经切到 Offboard（本模块不主动切）
		//   pos_valid —— position 型 Offboard 的前提（EKF 位置可用）
		if (armed.armed && offboard && pos_valid) {
			// 记录"地面参考"：NED 里 z 向下，所以地面就是进入 Offboard 时的 z。
			// 目标高度 = 地面 z - MYM_ALT（减去一个正数 = 往上）。
			_ground_z = pos.z;
			_target(2) = _ground_z - _param_mym_alt.get();

			// 锁死这一刻的航向。_yaw_sp 是 holdCurrentPosition() 每拍从姿态里抄的，
			// 出了 WAIT 就没人再刷新它（只有 WAIT/DONE 调 holdCurrentPosition），
			// 所以它天然就是"起飞时的机头方向"，后面 Forward 直接拿来用。
			// 记一份到 _takeoff_yaw 只是为了让 TAKEOFF 分支读起来不用去猜 _yaw_sp 的生命周期。
			_takeoff_yaw = _yaw_sp;

			_state = State::Takeoff;
			_state_start = now;
			PX4_INFO("my_mission: TAKEOFF -> %.2f m (yaw %.0f deg)", (double)_param_mym_alt.get(),
				 (double)math::degrees(_takeoff_yaw));
		}

		break;

	case State::Takeoff:
		// 每拍都发同一个目标位置。位置环会把它自己的速度/加速度限制加进来，
		// 所以"一次阶跃 1 m"不会真的瞬移，而是受控爬升。
		publishTrajectorySetpoint(now);

		if (reachedTarget(pos)) {
			const float fwd_dist = _param_mym_fwd_dist.get();

			if (fwd_dist > 0.f) {
				// ===========================================================
				// 算"前方那一个点"
				// ===========================================================
				// 航向 psi 的**机头单位向量在 NED 里就是 (cos psi, sin psi)**。
				// 原因：psi 的定义是"从北起、顺时针为正"，而 NED 的 x 指北、y 指东，
				// 所以 x->y 正好也是顺时针 —— 两个角度系统重合，不用转负号。
				// （对比 FRD 的 body x 轴，那个才要 (cos psi, sin psi, ...) 配合 z 向下。）
				const float psi = _takeoff_yaw + math::radians(_param_mym_fwd_dir.get());

				// 起点用 _target(0)/(1)：TAKEOFF 全程没动过它们，还是 WAIT 里钉下的
				// 起飞点。这样"飞到哪个点"相对起飞点固定，同一个场地跑几次都落在同一处。
				_target(0) += fwd_dist * cosf(psi);
				_target(1) += fwd_dist * sinf(psi);

				// z 不动：先在 MYM_ALT 上平飞过去，到了再降。
				_state = State::Forward;
				_state_start = now;
				PX4_INFO("my_mission: FORWARD %.2f m at %.0f deg (heading %.0f deg) -> target [%.2f %.2f]",
					 (double)fwd_dist, (double)_param_mym_fwd_dir.get(),
					 (double)math::degrees(_takeoff_yaw), (double)_target(0), (double)_target(1));

			} else {
				// MYM_FWD_DIST = 0：跳过 FORWARD，行为跟没有这个功能时完全一样。
				_state = State::Hover;
				_state_start = now;
				PX4_INFO("my_mission: HOVER %.1f s", (double)_param_mym_hover.get());
			}

		} else if (elapsedSince(_state_start, now) > _param_mym_tko_tmo.get()) {
			// 爬不上去（推力不够 / 位置估计漂 / MYM_ARRIVE 设太小）→ 安全起见直接转降落，
			// 不要无限期悬在那里等。
			PX4_WARN("my_mission: takeoff timeout, going to LAND");
			_state = State::Land;
			_state_start = now;
			_land_cmd_sent = false;
		}

		break;

	case State::Forward:
		// 目标在 TAKEOFF 分支里算好了，这里每拍照发（不发就是 offboard signal lost）。
		// 注意 yaw 保持锁存值不动 —— setpoint 里给了 yaw，位置环会用它当机头指向，
		// 于是飞机是"机头朝前平飞过去"，而不是侧着飘过去。
		publishTrajectorySetpoint(now);

		if (reachedTarget(pos)) {
			_state = State::Hover;
			_state_start = now;
			PX4_INFO("my_mission: reached forward point [%.2f %.2f], HOVER %.1f s",
				 (double)_target(0), (double)_target(1), (double)_param_mym_hover.get());

		} else if (elapsedSince(_state_start, now) > _param_mym_fwd_tmo.get()) {
			// 飞不到（航向估计歪 / 顶风 / EKF 位置漂 / MYM_ARRIVE 太小）→ 不再硬追，
			// 和起飞超时同样的策略：直接进降落流程，别一直在外面飞。
			PX4_WARN("my_mission: forward timeout, going to LAND");
			_state = State::Land;
			_state_start = now;
			_land_cmd_sent = false;
		}

		break;

	case State::Hover:
		// 目标不变（起飞点上空，或者有 MYM_FWD_DIST 时是前方那个点），位置环自己维持。
		// 注意这里**必须继续发** setpoint：不发就是 offboard signal lost，飞机会按 COM_OBL_RC_ACT 切走。
		publishTrajectorySetpoint(now);

		if (elapsedSince(_state_start, now) >= _param_mym_hover.get()) {
			_state = State::Land;
			_state_start = now;
			_land_cmd_sent = false;   // 允许在 LAND 里发一次降落命令
			PX4_INFO("my_mission: LAND (mode %d)", (int)_param_mym_land_mode.get());
		}

		break;

	case State::Land:
		if (_param_mym_land_mode.get() == 1) {
			// ---- 方式 1：交给 PX4 的 AUTO_LAND ----
			// 发一次 MAV_CMD_NAV_LAND，commander 会强制切到 NAVIGATION_STATE_AUTO_LAND
			// （Commander.cpp:1236-1247，注意那里 force = true，所以随时可切）。
			// 之后的收油门 / flare / 着陆检测 / 按 COM_DISARM_LAND 自动上锁
			// 全部由 PX4 负责，比自己在 Offboard 里硬降稳得多。
			//
			// 只发一次：这是"切模式"命令，不是连续控制量，每拍重发没有意义。
			if (!_land_cmd_sent) {
				publishVehicleCommand(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
				_land_cmd_sent = true;
			}

			// 这里**不再发 trajectory_setpoint**：nav_state 已经不是 Offboard，
			// 发了也没人看，留着只会让人误以为还在自己控。
			if (!armed.armed) {
				// PX4 自己上锁了（COM_DISARM_LAND，默认 2 s），省掉我们的 DISARM 步骤
				_state = State::Done;
				PX4_INFO("my_mission: landed + disarmed");

			} else if (land.landed && _param_mym_auto_disarm.get() != 0) {
				// 已经判到着地，主动补一发 disarm（不等 PX4 那个 2 s）
				_state = State::Disarm;
				_state_start = now;
				_last_disarm_cmd = 0;

			} else if (elapsedSince(_state_start, now) > _param_mym_land_tmo.get()) {
				// 降不下去 / 着地检测一直不触发：退一步，直接尝试上锁并结束
				PX4_WARN("my_mission: land timeout, trying to disarm");
				_state = State::Disarm;
				_state_start = now;
				_last_disarm_cmd = 0;
			}

		} else {
			// ---- 方式 0：纯 Offboard 下降 ----
			// 自己把目标 z 以 MYM_DESC_RATE 一帧一帧地往"地面参考"抬。
			// NED 里下降 = z 变大，所以是 + rate*dt，上限是 _ground_z。
			// 用斜坡而不是直接跳到地面：位置环虽然也有速度限幅，但自己限速更可控，
			// status 里看到的 setpoint 也是平滑的。
			//
			// 同时把 rate 当作 velocity[2] 前馈一起发出去 —— 这是让 land detector
			// 能判"被指令下降"的唯一方式（MulticopterLandDetector.cpp:248），
			// 没有它 landed 永远是 0，后面的 disarm 一定被 commander 拒。
			// 下限是 1.1 * LNDMC_Z_VEL_MAX（默认 0.25 -> 0.275 m/s）：
			// 低于它 _in_descend 就恒为 false，所以这里把参数抬上去并告警一次。
			float desc_rate = _param_mym_desc_rate.get();

			if (desc_rate < _in_descend_min_vel) {
				if (!_desc_rate_warned) {
					_desc_rate_warned = true;
					PX4_WARN("my_mission: MYM_DESC_RATE %.2f < %.2f (1.1*LNDMC_Z_VEL_MAX): raised, otherwise landing is undetectable",
						 (double)desc_rate, (double)_in_descend_min_vel);
				}

				desc_rate = _in_descend_min_vel;
			}

			if (pos_valid) {
				_target(2) = math::min(_target(2) + desc_rate * dt, _ground_z);
			}

			publishTrajectorySetpoint(now, desc_rate);

			if (!armed.armed) {
				_state = State::Done;

			} else if (land.landed || land.maybe_landed) {
				// **以 land detector 为准**收尾。
				// 不能用"目标 z 到了 + vz 小了"这种宽松判据：那只是"物理上贴地了"，
				// commander 的 disarm 检查要的是 land detector 的 landed/maybe_landed。
				// 我们一直在发 velocity[2] 前馈，所以 detector 会在触地后约 1 秒内
				// 依次满足 has_low_throttle -> ground_contact -> maybe_landed -> landed。
				// 兜底是 MYM_LAND_TMO（默认 40 s），超时会走上面那条超时分支。
				if (_param_mym_auto_disarm.get() != 0) {
					_state = State::Disarm;
					_state_start = now;
					_last_disarm_cmd = 0;
					PX4_INFO("my_mission: land detector confirmed (%s), disarming",
						 land.landed ? "landed" : "maybe_landed");

				} else {
					_state = State::Done;
				}
			}
		}

		break;

	case State::Disarm:

		// MAV_CMD_COMPONENT_ARM_DISARM 的 param1：1 = arm，0 = disarm。
		// 这里 param1 = 0。param2 保持 0 = 普通上锁：PX4 会拒绝"空中上锁"，
		// 只有 param2 = 21196（force）才允许在空中强行断动力 —— 那是紧急手段，
		// 这里不用。
		if (!armed.armed) {
			_state = State::Done;
			PX4_INFO("my_mission: DISARMED");

		} else {
			// 1 Hz 重试：commander 可能因为"还没判到着地"拒绝一次，过一会再试。
			if (_last_disarm_cmd == 0 || elapsedSince(_last_disarm_cmd, now) > 1.f) {
				publishVehicleCommand(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);
				_last_disarm_cmd = now;
			}

			if (elapsedSince(_state_start, now) > _param_mym_disarm_tmo.get()) {
				PX4_WARN("my_mission: disarm timeout, giving up");
				_state = State::Done;
			}
		}

		break;

	case State::Done:
		// 任务结束。仍然把 setpoint 钉在当前位置：如果飞手这时又切进 Offboard，
		// 位置环的目标就是当前位置，不会飞回旧的悬停点。
		holdCurrentPosition(pos, att, pos_valid);
		publishTrajectorySetpoint(now);

		// 已上锁 → 回到 WAIT：下一次 arm + 切 Offboard 会自动重跑整个流程。
		if (!armed.armed) {
			_state = State::Wait;
			_takeover_warned = false;   // 允许下一轮再告警
			_state_start = now;
		}

		break;
	}
}

int MyMission::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MyMission::print_status()
{
	PX4_INFO("state: %s", stateName(_state));
	PX4_INFO("target: [%.2f %.2f %.2f]  yaw %.3f  valid %d  ground_z %.2f",
		 (double)_target(0), (double)_target(1), (double)_target(2), (double)_yaw_sp, _target_valid, (double)_ground_z);
	PX4_INFO("land_mode %d  auto_disarm %d  alt %.2f m  hover %.1f s",
		 (int)_param_mym_land_mode.get(), (int)_param_mym_auto_disarm.get(),
		 (double)_param_mym_alt.get(), (double)_param_mym_hover.get());
	PX4_INFO("forward %.2f m  dir %.0f deg  tmo %.1f s  takeoff_yaw %.0f deg",
		 (double)_param_mym_fwd_dist.get(), (double)_param_mym_fwd_dir.get(),
		 (double)_param_mym_fwd_tmo.get(), (double)math::degrees(_takeoff_yaw));

	// 接管安全：现在把模式开关拨出 Offboard 会不会掉高度？
	manual_control_setpoint_s manual_control{};
	_manual_control_sub.copy(&manual_control);

	const bool centered = throttleStickCentered(manual_control);

	PX4_INFO("throttle %.2f -> %s (MAN_DEADZONE %.2f)  takeover_warned %d",
		 (double)manual_control.throttle,
		 centered ? "CENTERED (Position/Altitude will hold altitude)"
		 : "OFF-CENTER (Position/Altitude will descend immediately)",
		 (double)_man_deadzone, (int)_takeover_warned);

	return 0;
}

int MyMission::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Offboard position-control mission demo for a multicopter:

  WAIT (armed + Offboard) -> TAKEOFF (MYM_ALT)
  -> FORWARD (MYM_FWD_DIST along the takeoff heading, skipped when 0)
  -> HOVER (MYM_HOVER) -> LAND (offboard descent or PX4 AUTO_LAND) -> DISARM -> DONE

Publishes offboard_control_mode{position=true} and trajectory_setpoint at 50 Hz,
and uses vehicle_command for landing/disarming.

### Examples
Start the module, then arm and switch to Offboard:
$ my_mission start
$ commander arm
$ commander mode offboard

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_mission", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int MyMission::task_spawn(int argc, char *argv[])
{
	MyMission *instance = new MyMission();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int my_mission_main(int argc, char *argv[])
{
	return ModuleBase::main(MyMission::desc, argc, argv);
}
