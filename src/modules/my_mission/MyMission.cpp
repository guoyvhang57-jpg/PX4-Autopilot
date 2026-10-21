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
 * 任务编排：状态机 + 9 个状态函数。
 *
 * Offboard 是什么、三条 topic 怎么配合、飞手要按什么顺序操作、每个参数什么含义，
 * 全部在 docs/offboard.md 和 README.md 里，本文件只管"什么时候做什么"。
 *
 * 刻意留下的两条规矩：
 *   1) **只有 MissionOutput 发 topic。** trajectory_setpoint 是共享 topic，
 *      非 Offboard 时发它会把 land detector 搞坏（见 MissionOutput.cpp）。
 *   2) **状态函数自己改 _state / _state_start。** 不统一走"返回下一个状态"，
 *      因为现有代码里有几处迁移顺带做了别的事，而且 Disarm→Done 是故意不刷
 *      _state_start 的。保持"纯搬迁"，行为一个字节都不变。
 */

#include "MyMission.hpp"

#include <mathlib/mathlib.h>

#include <lib/geo/geo.h>   // CONSTANTS_ONE_G：算"加速度对应多大倾角"用

#include <px4_platform_common/log.h>

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

	refreshConfig();
	_external.refresh();

	return true;
}

const char *MyMission::stateName(State state)
{
	switch (state) {
	case State::Wait:    return "WAIT";

	case State::Takeoff: return "TAKEOFF";

	case State::Forward: return "FORWARD";

	case State::Tilt:    return "TILT";

	case State::Hover:   return "HOVER";

	case State::Return:  return "RETURN";

	case State::Land:    return "LAND";

	case State::Disarm:  return "DISARM";

	case State::Done:    return "DONE";

	case State::Count:   break;
	}

	return "?";
}

bool MyMission::localPositionValid(const vehicle_local_position_s &pos, hrt_abstime now)
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
	return (p - _target).norm() < _config.arrive && v.norm() < 0.3f;
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

MissionOutput::Setpoint MyMission::targetSetpoint() const
{
	MissionOutput::Setpoint setpoint;

	// 还没有有效位置估计 → 位置保持 NaN（= 不控制位置），
	// 千万不要发 (0,0,0)：那会被理解成"飞到 EKF 原点"。
	// Setpoint 的默认值就是全 NaN，所以这里只在有效时覆盖。
	if (_target_valid) {
		setpoint.position = _target;
	}

	setpoint.yaw = _yaw_sp;

	return setpoint;
}

void MyMission::refreshConfig()
{
	_config.alt = _param_mym_alt.get();
	_config.fwd_dist = _param_mym_fwd_dist.get();
	_config.fwd_dir_deg = _param_mym_fwd_dir.get();
	_config.fwd_spd = _param_mym_fwd_spd.get();
	_config.fwd_acc = _param_mym_fwd_acc.get();
	_config.fwd_jerk_max = _param_mym_fwd_jerk_max.get();
	_config.fwd_jerk_min = _param_mym_fwd_jerk_min.get();
	_config.fwd_tmo = _param_mym_fwd_tmo.get();
	_config.hover = _param_mym_hover.get();
	_config.land_mode = _param_mym_land_mode.get();
	_config.desc_rate = _param_mym_desc_rate.get();
	_config.arrive = _param_mym_arrive.get();
	_config.tko_tmo = _param_mym_tko_tmo.get();
	_config.land_tmo = _param_mym_land_tmo.get();
	_config.auto_disarm = (_param_mym_auto_disarm.get() != 0);
	_config.disarm_tmo = _param_mym_disarm_tmo.get();

	_config.pitch_down_deg = _param_myr_pitch_down.get();
	_config.tilt_hold = _param_myr_tilt_hold.get();
	_config.fwd_thr = _param_myr_fwd_thr.get();
}

// ===========================================================================
// 中转：FORWARD 和 ROPE 共用的"用 TransitLeg 飞到 _goal3"
// ===========================================================================
TransitLeg::Limits MyMission::transitLimits() const
{
	TransitLeg::Limits limits;

	limits.cruise_speed = _config.fwd_spd;
	limits.accel_xy = _config.fwd_acc;
	limits.vel_xy_max = _external.mpcXyVelMax();
	// 这里给的是"进中转那一刻"的 jerk（离限幅远的大值）。真正每拍用多少由
	// advanceTransit() 里的 scheduledJerk() 下发：按需求姿态离低头限幅的比例
	// 在 MYM_FWD_JERK_MAX 和 MYM_FWD_JERK_MIN 之间插值。
	limits.jerk = _config.fwd_jerk_max;
	limits.err_max_xy = _external.mpcXyErrMax();
	limits.err_max_z = _external.mpcZErrMax();
	limits.vel_z_max = _external.mpcZvAutoUp();
	limits.accel_z_max = _external.mpcAccUpMax();

	return limits;
}

void MyMission::enterTransit(const Tick &tick, State next)
{
	_transit.setLimits(transitLimits());
	_transit_acc_xy = 0.f;    // 调度从"平飞"开始，第一拍用 MYM_FWD_JERK_MAX
	_transit_acc_along = 0.f;
	_decel_timer = 0.f;

	// 锁存中转航向的水平单位向量，用来把加速度投影成"带符号的沿航向分量"。
	// 距离太近（< 0.5 m）时方向没有意义，退回起飞航向。
	const matrix::Vector2f to_goal(_goal3(0) - tick.pos.x, _goal3(1) - tick.pos.y);
	_transit_dir = (to_goal.norm() > 0.5f) ? to_goal.unit()
		       : matrix::Vector2f(cosf(_takeoff_yaw), sinf(_takeoff_yaw));

	_transit.start(_goal3, tick.pos);

	enterState(next, tick.now);
}

float MyMission::scheduledJerk() const
{
	// 低头限幅的口径和 PositionControl 完全一致：MYM_PITCH_MIN（负值）和
	// MPC_TILTMAX_AIR（正值）里更严的那个，换算成"低头角的绝对值"。
	float ceiling_deg = _external.mpcTiltmaxAir();
	const float assist_ceiling_deg = -_external.mymPitchMin();   // MYM_PITCH_MIN < 0

	if (assist_ceiling_deg > 0.f) {
		ceiling_deg = math::min(ceiling_deg, assist_ceiling_deg);
	}

	const float jmax = math::max(_config.fwd_jerk_max, 0.1f);

	if (ceiling_deg < 0.1f) {
		return jmax;   // 没设低头限幅，没有可逼近的目标
	}

	// 需求姿态 = atan(a_h / g)。a_h 用上一拍实际发出去的水平加速度前馈。
	const float demand_deg = math::degrees(atanf(_transit_acc_xy / CONSTANTS_ONE_G));

	// 从限幅的这个比例处才开始降 jerk，而不是从 0 就开始降。
	//
	// 从 0 开始降的版本实测过：需求角会渐近地逼近限幅（jerk 越小、逼近越慢），
	// 而轨迹的"速度预算"在同一段时间里被消耗掉 —— MYM_FWD_ACC=4、巡航 6 m/s、
	// 40 m 的航段上，加速度前馈只爬到 2.53 m/s^2，需求角停在 14.5 度，
	// 缺额恒为零，推进电机一次都没转起来。
	// 留出前 80% 的"不干预区"之后，需求角是快速穿过限幅的，穿过之后
	// 按 MYM_FWD_JERK_MIN 往上走，前推力这才有得补。
	static constexpr float kScheduleStartRatio = 0.8f;

	const float ratio = math::constrain((demand_deg / ceiling_deg - kScheduleStartRatio)
					    / (1.f - kScheduleStartRatio), 0.f, 1.f);
	const float jmin = math::constrain(_config.fwd_jerk_min, 0.1f, jmax);

	// 线性插值，不是阈值开关：开关会让 jerk 在相邻两拍之间反复跳，
	// 规划出来的加速度斜率也跟着跳，等于自己制造抖动。
	return jmax + (jmin - jmax) * ratio;
}

bool MyMission::advanceTransit(const Tick &tick)
{
	// 每拍重设一次 jerk：平滑器只在下一次重规划里用它，所以从当前状态往后
	// 按新限幅走，已经走过的轨迹不动，加速度不会出现跳变。
	_transit.setJerk(scheduledJerk());

	const TransitLeg::Update u = _transit.update(tick.pos, tick.dt);

	_transit_acc_xy = matrix::Vector2f(u.acceleration(0), u.acceleration(1)).norm();

	// ---- 减速段判定 ----
	// 沿航向的**有符号**加速度翻负并持续 0.2 s，就认为轨迹进入减速段。
	// 单向闩锁：一段航程里减速之后不会再加速，所以没有退出条件，也就不会在零点抖。
	// 闩锁之后 Run() 把前推力辅助关掉 —— 减速段姿态抬头，夹紧没有力可以交出去，
	// 而减速过程中速度掉到参考以下时需求会短暂翻回低头，那一瞬间辅助会重新介入。
	{
		const matrix::Vector2f acc_xy(u.acceleration(0), u.acceleration(1));
		_transit_acc_along = acc_xy.dot(_transit_dir);

		if (!_decel_latched && (_transit_acc_along < -0.2f)) {
			_decel_timer += tick.dt;

			if (_decel_timer > 0.2f) {
				_decel_latched = true;
				PX4_INFO("my_mission: deceleration phase (a_along %.2f m/s^2), forward assist off",
					 (double)_transit_acc_along);
			}

		} else if (_transit_acc_along >= -0.2f) {
			_decel_timer = 0.f;
		}
	}

	// 位置、速度、**加速度**三个量一起发。加速度这一项是修掉末端过冲的关键：
	// PositionControl.cpp:153 把它直接加进加速度指令，绕过了速度环那个
	// τ ≈ 0.56 s 的一阶滞后。没有它，参考做匀速减速时飞机永远比参考快 a·τ，
	// 到终点时早就冲过去了（实测 0.39~0.45 m）。
	MissionOutput::Setpoint setpoint;
	setpoint.position = u.position;
	setpoint.velocity_xy = u.velocity_xy;
	setpoint.acceleration = u.acceleration;
	setpoint.yaw = _yaw_sp;
	publishSetpoint(setpoint, tick.now);

	// 让 HOVER / LAND / print_status 接着用同一个位置
	_target = u.position;
	_goal_distance = u.goal_distance;

	// 到位判据两件事都要满足：
	//   1) **轨迹**已经贴住终点。平滑器的速度上限是按剩余距离反解的，
	//      所以"位置到终点"和"速度归零"本来就是同一件事（TrajMath.hpp:61-71）。
	//   2) **飞机**的速度也小。轨迹到了不等于飞机到了，中间还有跟踪误差。
	const matrix::Vector3f v(tick.pos.vx, tick.pos.vy, tick.pos.vz);

	return _transit.arrived(_config.arrive) && (v.norm() < 0.3f);
}

// ===========================================================================
// 系绳测试
// ===========================================================================
void MyMission::logTransitStart(const Tick &tick)
{
	const matrix::Vector3f here(tick.pos.x, tick.pos.y, tick.pos.z);
	const float dist = (_goal3 - here).norm();
	const float climb = tick.pos.z - _goal3(2);   // NED：正 = 要往上爬
	const float t_est = TransitLeg::estimateDuration(dist, _config.fwd_spd, _config.fwd_acc);

	PX4_INFO("my_mission: FORWARD %.2f m at %.0f deg (heading %.0f deg) -> target [%.2f %.2f]",
		 (double)_config.fwd_dist, (double)_config.fwd_dir_deg,
		 (double)math::degrees(_takeoff_yaw), (double)_goal3(0), (double)_goal3(1));

	PX4_INFO("my_mission: %.1f m to go (climb %.1f m), cruise %.2f m/s acc %.2f m/s^2 jerk %.2f, "
		 "est %.1f s (tilt ~%.0f deg)",
		 (double)dist, (double)climb, (double)_config.fwd_spd, (double)_config.fwd_acc,
		 (double)_config.fwd_jerk_max, (double)t_est,
		 (double)math::degrees(atanf(_config.fwd_acc / CONSTANTS_ONE_G)));

	if (t_est > _config.fwd_tmo) {
		PX4_WARN("my_mission: est %.1f s > MYM_FWD_TMO %.1f s -> will time out; "
			 "raise MYM_FWD_TMO or MYM_FWD_SPD", (double)t_est, (double)_config.fwd_tmo);
	}
}

// ===========================================================================
// 每拍入口：退出协议 -> 取数 -> 安全兜底 -> 分发
// ===========================================================================
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

	// dt 用实测时间差，不再写死 0.02。两个理由：
	//   1) PositionSmoothing 靠它推进轨迹（内部是 `_local_time += dt * stretch`），
	//      dt 不准会让轨迹的"时间"和"几何"对不上。PX4 自己也这么做，而且同样把
	//      结果夹在 2~40 ms 之间（MulticopterPositionControl.cpp:391-392）——
	//      上界防一次卡顿把轨迹推飞，下界防除零。
	//   2) 方式 0 的下降斜坡也靠它积分，调度抖动会让下降速率偏。
	const float dt = (_last_run == 0) ? 0.02f
			 : math::constrain((now - _last_run) * 1e-6f, 0.002f, 0.04f);
	_last_run = now;

	// 参数在线更新：QGC 或命令行改完 MYM_* / MYR_* 立即生效，不用重启模块、更不用重编。
	if (_parameter_update_sub.updated()) {
		updateParams();
		refreshConfig();
		_external.refresh();
	}

	// ---- 取数 ----
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

	// MissionOutput 用它决定"这一拍该不该发 setpoint"，
	// 不能乱发的原因见 MissionOutput::publishSetpoint 开头。
	_output.setOffboard(offboard);

	// ---- 1) 先发 offboard_control_mode ----
	// 放在状态机之前、而且**任何状态都发**，三个原因：
	//   a) 切进 Offboard 之前就得让 commander 看到信号，否则模式开关切不进去；
	//   b) 进去之后不能断，断了就是 offboard loss failsafe；
	//   c) WAIT / DONE 也可能正处在 Offboard 里（比如刚上锁还没切走），一样要续命。
	publishControlMode(now);

	// ---- 前推力辅助的阶段许可 ----
	// 每拍都发（含 WAIT/DONE）：控制器用 500 ms 超时判活，断流按"禁止"处理。
	// 只在"需要往前加速"的时候开：
	//   TILT     —— 整个拉绳段都要，这是它存在的意义
	//   FORWARD  —— 加速段和巡航段要，**减速段关掉**（姿态抬头，没有力要交出去）
	//   其余     —— RETURN 是空载回家，HOVER/LAND 更不需要，一律关
	const bool assist_allowed = (_state == State::Tilt)
				    || ((_state == State::Forward) && !_decel_latched);
	_output.publishAssist(assist_allowed, now);

	// ---- 2) 安全兜底 ----
	// 只有"正在起飞 / 前飞 / 绳段 / 低头 / 悬停"时，掉出 Offboard 或上锁才算异常，
	// 直接中止。LAND 阶段允许离开 Offboard —— 方式 1 的 AUTO_LAND 本来就会把
	// nav_state 切走；DISARM / DONE 本来就已经落地或结束，不算异常。
	if ((_state == State::Takeoff || _state == State::Forward
	     || _state == State::Tilt || _state == State::Hover || _state == State::Return)
	    && (!offboard || !armed.armed)) {
		PX4_WARN("my_mission: abort (offboard=%d armed=%d)", (int)offboard, (int)armed.armed);
		enterState(State::Done, now);
	}

	// ---- 3) 接管安全：离开 Offboard 后油门没回中 ----
	// 放在 Done 的头 1 秒内检查，而不是上面那个 if 里：nav_state 的更新比
	// mode 开关晚 1~2 拍，在 Done 里再看几拍容错更大。
	// armed 才检查 —— 如果是被 kill/termination 上锁退出的，不会再有掉高度的问题。
	if (!_takeover_warned && (_state == State::Done) && armed.armed && (elapsedSince(_state_start, now) < 1.f)) {
		_takeover_warned = _takeover.evaluate(status, manual_control, _external.manDeadzone());
	}

	// ---- 4) 分发 ----
	const Tick tick{now, dt, pos, att, armed, land, pos_valid, offboard};

	static constexpr Handler kHandlers[] = {
		&MyMission::stateWait, &MyMission::stateTakeoff, &MyMission::stateForward,
		&MyMission::stateTilt, &MyMission::stateHover, &MyMission::stateReturn,
		&MyMission::stateLand, &MyMission::stateDisarm, &MyMission::stateDone,
	};
	static_assert(sizeof(kHandlers) / sizeof(kHandlers[0]) == int(State::Count), "state table out of sync");

	(this->*kHandlers[int(_state)])(tick);
}

// ===========================================================================
// 状态函数
// ===========================================================================

/**
 * @brief TILT 加速度前馈的斜坡速率 [m/s^2 per s]
 *
 * **写死，不做参数**：它只影响"进入和离开低头那一刻"的平滑度，和飞行性能无关，
 * 现场没有调它的理由，多一个参数只是多一个能设错的地方。
 *
 * 取 2.5 的依据：15° 的低头对应 a = g*tan(15°) = 2.63 m/s^2，用 1.05 s 建立，
 * 也就是姿态平均约 14 °/s。阶跃时实测进入是 64 °/s、离开 73 °/s 的角速度尖峰，
 * 斜坡把它压掉一个量级，而且姿态是"爬上去/退下来"，不是被推一把。
 */
static constexpr float kTiltFfSlew = 2.5f;

void MyMission::stateWait(const Tick &tick)
{
	// 进 Offboard 之前：目标 = 当前位置。位置环就算已经在跑也不会动。
	holdCurrentPosition(tick.pos, tick.att, tick.pos_valid);
	publishSetpoint(targetSetpoint(), tick.now);

	// ---- 地面参考 ----
	// 必须在**还没起飞**、也就是 land detector 说"在地上"的时候锁存。
	// 手动进场那条路会跳过 TAKEOFF，等它切进 Offboard 时飞机已经在空中了，
	// 那时再把当前 z 当成地面，Offboard 下降（MYM_LAND_MODE=0）就永远落不下去。
	if (!_ground_z_latched && tick.pos_valid && tick.land.landed) {
		_ground_z = tick.pos.z;
		_ground_z_latched = true;
	}

	// 三个条件齐了才进下一步：
	//   armed     —— 电机允许转
	//   offboard  —— 飞手已经切到 Offboard（本模块不主动切）
	//   pos_valid —— position 型 Offboard 的前提（EKF 位置可用）
	if (tick.armed.armed && tick.offboard && tick.pos_valid) {
		const matrix::Vector3f here(tick.pos.x, tick.pos.y, tick.pos.z);

		// ===============================================================
		// 统一参考点：PX4 的 home_position，也就是**解锁那一刻**的位置
		// ===============================================================
		//    x/y —— TAKEOFF 的起飞点、FORWARD 的起点（MYM_FWD_DIST 从这里算）
		//    z   —— 地面参考，MYM_LAND_MODE=0 的下降目标
		//    距离 —— 手动进场的判据
		//    RETURN 的目标 —— 同一组 x/y
		//
		// 这样整趟任务只有一个坐标原点到"地面上的哪个点"的映射，不会出现
		// "起飞点飞出去、返航回解锁点"这种分叉。
		// home_position.x/y/z 本来就是本地系坐标，直接拿来用，不用投影。
		// home 无效（没解锁过 / 话题没来）时退回"当前位置 + 锁存的地面"，和原来一致。
		home_position_s home{};
		const bool home_valid = _home_position_sub.copy(&home) && home.valid_lpos;
		const matrix::Vector3f home_pos = home_valid
						  ? matrix::Vector3f(home.x, home.y, home.z)
						  : matrix::Vector3f(here(0), here(1),
								  _ground_z_latched ? _ground_z : here(2));
		_ground_z = home_pos(2);
		_ground_z_latched = true;

		const float dist_home = (here - home_pos).norm();

		// 锁死这一刻的航向。_yaw_sp 是 holdCurrentPosition() 每拍从姿态里抄的，
		// 出了 WAIT 就没人再刷新它（只有 WAIT/DONE 调 holdCurrentPosition），
		// 所以它天然就是"此刻的机头方向"，TILT / FORWARD 直接拿来用。
		_takeoff_yaw = _yaw_sp;

		// ===============================================================
		// 手动进场：开着绳 + 已经离地 + 离原点超过 MYM_ARRIVE
		// ===============================================================
		// 说明飞手是**手动**把飞机飞到"绳快要绷紧"的地方才切进 Offboard 的，
		// 那就不要爬升、不要飞中转段，直接进 TILT 开始拉。
		//
		// ⚠ 必须要求 !landed：地上的飞机被人挪过位置（离原点 > MYM_ARRIVE）时，
		// 一进 Offboard 就会坐在地上拉绳，会翻。
		// 只要**离地之后**才切进 Offboard，就认定是拉绳测试。
		// 地面 arm 完直接切 -> 走下面的正常起飞流程。
		if (!tick.land.landed && (dist_home > _config.arrive)) {
			// 三轴都用当前值：手动飞过来的高度就是拉绳高度，不套 MYM_ALT。
			// （拉绳点就是飞手停的那个地方，这是唯一一个**有意**不以 home 为准的
			//   位置 —— 它的定义就是"飞机现在在哪"。）
			_target = here;
			_target_valid = true;
			_tilt_ff = 0.f;
			_last_rope_log = 0;
			enterState(State::Tilt, tick.now);
			PX4_INFO("my_mission: manual entry %.2f m from %s (%.2f m above ground) -> PULL %.1f deg, %.1f s",
				 (double)dist_home, home_valid ? "home" : "current position",
				 (double)(_ground_z - here(2)),
				 (double)_config.pitch_down_deg, (double)_config.tilt_hold);

		} else {
			// 正常流程：在 home 正上方起飞，高度 = home 的地面 + MYM_ALT。
			// 后面 FORWARD 的终点也是从这里的 _target 加出去的，所以它自动变成
			// "从 home 往前走 MYM_FWD_DIST"。
			_target(0) = home_pos(0);
			_target(1) = home_pos(1);
			_target(2) = _ground_z - _config.alt;

			enterState(State::Takeoff, tick.now);
			PX4_INFO("my_mission: TAKEOFF at %s [%.2f %.2f] -> %.2f m (yaw %.0f deg)",
				 home_valid ? "home" : "current position",
				 (double)home_pos(0), (double)home_pos(1), (double)_config.alt,
				 (double)math::degrees(_takeoff_yaw));
		}
	}
}

void MyMission::stateTakeoff(const Tick &tick)
{
	// 每拍都发同一个目标位置。位置环会把它自己的速度/加速度限制加进来，
	// 所以"一次阶跃 1 m"不会真的瞬移，而是受控爬升。
	publishSetpoint(targetSetpoint(), tick.now);

	if (reachedTarget(tick.pos)) {
		// ===============================================================
		// TAKEOFF 到位后只有一个去向：前方点
		// ===============================================================
		// 拉绳测试**不走这条路** —— 它只从"手动起飞、飞出去、再切 Offboard"
		// 进入（见 stateWait）。地面起飞的这一趟和有没有开绳无关，
		// 开绳只是让 WAIT 那边允许手动进场。
		if (_config.fwd_dist > 0.f) {
			// ===========================================================
			// 算"前方那一个点"
			// ===========================================================
			// 航向 psi 的**机头单位向量在 NED 里就是 (cos psi, sin psi)**。
			// 原因：psi 的定义是"从北起、顺时针为正"，而 NED 的 x 指北、y 指东，
			// 所以 x->y 正好也是顺时针 —— 两个角度系统重合，不用转负号。
			// （对比 FRD 的 body x 轴，那个才要 (cos psi, sin psi, ...) 配合 z 向下。）
			const float psi = _takeoff_yaw + math::radians(_config.fwd_dir_deg);

			// 起点用 _target(0)/(1)：TAKEOFF 全程没动过它们，还是 WAIT 里钉下的
			// 起飞点。这样"飞到哪个点"相对起飞点固定，同一个场地跑几次都落在同一处。
			//
			// 【关键】这里只把**终点**存进 _goal3，**_target 一个数都不改**。
			// 以前是直接 _target += fwd_dist*dir，那是给位置环一个阶跃，
			// 位置环下一拍就会要 (阶跃 x MPC_XY_P) 那么大的速度、姿态猛压。
			// 现在 _target 留在起飞点，由 FORWARD 状态每拍推一点点过去。
			_goal3(0) = _target(0) + _config.fwd_dist * cosf(psi);
			_goal3(1) = _target(1) + _config.fwd_dist * sinf(psi);
			_goal3(2) = _target(2);   // z 不动：先在 MYM_ALT 上平飞过去

			enterTransit(tick, State::Forward);
			logTransitStart(tick);

		} else {
			// 既没开绳、MYM_FWD_DIST 也是 0：没有目的地，悬停 MYM_HOVER 秒后降落。
			enterState(State::Hover, tick.now);
			PX4_INFO("my_mission: HOVER %.1f s", (double)_config.hover);
		}

	} else if (elapsedSince(_state_start, tick.now) > _config.tko_tmo) {
		// 爬不上去（推力不够 / 位置估计漂 / MYM_ARRIVE 设太小）→ 安全起见直接转降落，
		// 不要无限期悬在那里等。
		PX4_WARN("my_mission: takeoff timeout, going to LAND");
		enterState(State::Land, tick.now);
		_land_cmd_sent = false;
	}
}

void MyMission::stateForward(const Tick &tick)
{
	// 每拍的轨迹生成 + 发布 + 到位判据都在 advanceTransit() 里。
	if (advanceTransit(tick)) {
		// ⚠ 切走之前把 _target **钉到真正的终点**，不要留在平滑器最后的位置上。
		// 到位判据带 MYM_ARRIVE 的容差（默认 0.2 m），而平滑器是渐近收敛的 ——
		// 直接切走会让飞机永久停在离终点最多 0.2 m 的地方（实测 0.14 m）。
		// 钉到终点之后，下一个状态的位置环会把剩下那点走完。
		_target = _goal3;

		enterState(State::Hover, tick.now);
		PX4_INFO("my_mission: reached forward point [%.2f %.2f], HOVER %.1f s",
			 (double)_goal3(0), (double)_goal3(1), (double)_config.hover);

	} else if (elapsedSince(_state_start, tick.now) > _config.fwd_tmo) {
		// 飞不到（航向估计歪 / 顶风 / EKF 位置漂 / 限幅设得太小）→ 不再硬追，
		// 和起飞超时同样的策略：直接进降落流程，别一直在外面飞。
		PX4_WARN("my_mission: forward timeout (traj %.2f m from goal), going to LAND", (double)_goal_distance);
		enterState(State::Land, tick.now);
		_land_cmd_sent = false;
	}
}

void MyMission::stateTilt(const Tick &tick)
{
	// ===========================================================
	// 停住不动，低头 MYR_PITCH_DOWN，高度不变
	// ===========================================================
	// 本模块发的是 position 型 offboard，**姿态是位置环的输出**，没法直接指定。
	// 但加速度前馈会被原样加进加速度指令（PositionControl.cpp:153），
	// 而加速度到姿态的映射是（_accelerationControl，:222）
	//     body_z = -(a_x, a_y, g - a_z) 归一化
	// 也就是 平飞倾角 = atan(a / g)。所以要"低头 φ"，给 a = g*tan(φ) 就行。
	//
	// 为什么这样能**停住不动**：低头之后升力的水平分量把飞机往前推，但绳已经
	// 绷紧、拉住了，飞机几乎不前进 -> 位置误差≈0 -> 位置环和速度环不额外出力
	// -> 稳态姿态就是 φ。高度由 z 通道自己维持：倾角让升力的竖直分量变成
	// cos(φ) 倍，位置环会自动补总距；绳的竖直分量（T*sinθ，向下）也在这里
	// 被一并吸收，所以"高度不变"是自动成立的，不需要额外算。
	//
	// 稳态受力（拿来对表用的）：
	//     4*T_lift*cos(φ) = m*g + T_rope*sin(θ)
	//     T_rope*cos(θ)   = m*g*tan(φ) + 推力电机的推力
	// φ = 20° 时 T_rope ≈ 14.6 N（不含推力电机），φ = 30° 时 ≈ 23.1 N。
	// 插件的张力上限现在是 1000 N（按真绳承载给），这个量级连边都摸不到，
	// 所以实际约束不是上限而是 <stiffness>（3000 N/m：1 N 对应 0.33 mm 伸长）。
	// 巡航段以前实测会差 1~2 度（MYR_PITCH_DOWN=15 得到 13.5），原因是速度环
	// 积分器在前一段飞行的扰动里留了一点残余（约 -0.27 m/s²）。现在 TILT 直接接在
	// TAKEOFF 之后、没有中转段，这个残余应该小得多；`MYR_FWD_THR` 那一路也会
	// 吃掉一部分残差。
	float pitch_down = math::constrain(_config.pitch_down_deg, -_external.mpcTiltmaxAir(), _external.mpcTiltmaxAir());

	if ((fabsf(pitch_down - _config.pitch_down_deg) > 0.01f) && !_pitch_clamped_warned) {
		_pitch_clamped_warned = true;
		PX4_WARN("my_mission: MYR_PITCH_DOWN %.1f deg beyond MPC_TILTMAX_AIR %.1f deg: clamped",
			 (double)_config.pitch_down_deg, (double)_external.mpcTiltmaxAir());
	}

	// ---- 倾角 + 油门 -> 加速度前馈 ----
	// 和 PositionControl 那套互为逆运算：
	//     需求角 = atan(a / g)，夹在低头上限 ceiling
	//     缺额   = tan(需求角) - tan(ceiling)，乘 MYM_FWD_THR_SC 就是归一化前推力
	// 所以反过来解：
	//     油门 = 0  -> a = g * tan(MYR_PITCH_DOWN)                        姿态 = 你给的角度
	//     油门 > 0  -> a = g * ( tan(ceiling) + 油门 / SC )               姿态 = ceiling，推进电机 = 油门
	const float sc = _external.mymFwdThrSc();
	const float thr = math::constrain(_config.fwd_thr, 0.f, 1.f);

	float ceiling_deg = _external.mpcTiltmaxAir();
	const float assist_ceiling_deg = -_external.mymPitchMin();   // MYM_PITCH_MIN < 0

	if (assist_ceiling_deg > 0.f) {
		ceiling_deg = math::min(ceiling_deg, assist_ceiling_deg);
	}

	// 油门一给，姿态必然被夹在上限，倾角参数就没意义了 —— 这和"角度慢慢加"的
	// 用法一致：油门留 0，只调 MYR_PITCH_DOWN。
	const float pitch_used = (thr > 1e-3f) ? ceiling_deg
				 : math::min(pitch_down, ceiling_deg);

	if ((thr > 1e-3f) && (sc < 0.01f) && !_pitch_clamped_warned) {
		_pitch_clamped_warned = true;
		PX4_WARN("my_mission: MYR_FWD_THR %.2f needs MYM_FWD_THR_SC > 0 (got %.3f), throttle ignored",
			 (double)thr, (double)sc);
	}

	const float a_full = CONSTANTS_ONE_G * (tanf(math::radians(pitch_used))
						+ ((sc > 0.01f) ? thr / sc : 0.f));

	// ---- 前馈斜坡 ----
	// 保持时间内目标 = a_full；保持时间到了就把目标改成 0，先退回来再转降落。
	// 直接阶跃会在进入、离开各打一次 60~70 °/s 的角速度尖峰（实测），
	// 斜坡之后姿态是平滑地爬上去、退下来。
	const bool holding = elapsedSince(_state_start, tick.now) < _config.tilt_hold;
	const float a_target = holding ? a_full : 0.f;
	const float a_step = kTiltFfSlew * tick.dt;

	if (_tilt_ff < a_target) {
		_tilt_ff = math::min(_tilt_ff + a_step, a_target);

	} else if (_tilt_ff > a_target) {
		_tilt_ff = math::max(_tilt_ff - a_step, a_target);
	}

	// ---- 水平目标跟着飞机走 ----
	// 不跟的话位置环会把"低头把飞机推出去"的那段位移当成跟踪误差，反过来把前馈
	// 抵消掉：实测 a_ff = 1.73 m/s² 时，控制器输出的 acc_sp 只剩 0.05~0.14，
	// 姿态停在 0.2 度（本来要 10 度），飞机只有 0.8 m 的漂移就稳住了。
	// 跟着走之后位置误差≈0，需求就只剩下面前馈；速度环仍然把水平速度压到 0，
	// 所以飞机不会一直往前爬。
	//
	// z **不跟**：高度靠位置环维持，跟着走等于放弃高度控制。
	_target(0) = tick.pos.x;
	_target(1) = tick.pos.y;

	// 沿起飞航向的加速度前馈；z 给 NaN = 不做前馈，让位置环自己维持高度。
	MissionOutput::Setpoint setpoint = targetSetpoint();
	setpoint.acceleration = matrix::Vector3f(_tilt_ff * cosf(_takeoff_yaw), _tilt_ff * sinf(_takeoff_yaw), NAN);
	publishSetpoint(setpoint, tick.now);

	// 1 Hz 打一次实测的斜距 / 绳角 / 俯仰，和 gz 那边的张力遥测对表：
	//   gz topic -e -t /towed_0/tether     （x = 张力[N]，y = 斜距[m]，z = 绳角[deg]）
	if ((_last_rope_log == 0) || (elapsedSince(_last_rope_log, tick.now) >= 1.f)) {
		_last_rope_log = tick.now;

		const matrix::Vector3f here(tick.pos.x, tick.pos.y, tick.pos.z);

		// 实际俯仰角。Eulerf 的 theta 正 = 抬头，所以"低头角"要取负。
		const float pitch_real = -math::degrees(matrix::Eulerf(matrix::Quatf(tick.att.q)).theta());

		PX4_INFO("my_mission: TILT demand %.1f deg (a_ff %.2f, thr %.0f%%), actual %.1f deg, alt %.2f m",
			 (double)pitch_used, (double)_tilt_ff, (double)(thr * 100.f), (double)pitch_real,
			 (double)(-here(2)));
	}

	// 保持时间用 MYR_TILT_HOLD，**不是 MYM_HOVER** —— 低头保持多久和
	// "没开绳时到前方点悬停多久"是两件独立的事，共用一个参数会互相牵制。
	//
	// 注意退出条件是"时间到 **且** 前馈已经退回 0"：斜坡退回来还要
	// |a_full|/kTiltFfSlew 秒（15° 约 1.05 s），所以 TILT 的总时长比
	// MYR_TILT_HOLD 多出这一段。不能在还歪着的时候切走 —— 那正是原来
	// 73 °/s 尖峰的来源。
	if (!holding && (fabsf(_tilt_ff) < 1e-3f)) {
		enterState(State::Land, tick.now);
		_land_cmd_sent = false;
		PX4_INFO("my_mission: TILT done (%.1f s + ramp), LAND (mode %d)", (double)_config.tilt_hold,
			 _config.land_mode);
	}
}

void MyMission::stateHover(const Tick &tick)
{
	// 目标不变（起飞点上空，或者有 MYM_FWD_DIST 时是前方那个点），位置环自己维持。
	// 注意这里**必须继续发** setpoint：不发就是 offboard signal lost，
	// 飞机会按 COM_OBL_RC_ACT 切走。
	publishSetpoint(targetSetpoint(), tick.now);

	if (elapsedSince(_state_start, tick.now) >= _config.hover) {
		// 悬停结束**不直接降落**：先飞回 home（解锁时设的那个点），到了再落。
		// 高度取当前值 —— 回家是同高度平飞，不是降到地面。
		home_position_s home{};
		const bool home_valid = _home_position_sub.copy(&home) && home.valid_lpos;

		if (home_valid) {
			_home = matrix::Vector3f(home.x, home.y, tick.pos.z);
			enterState(State::Return, tick.now);
			PX4_INFO("my_mission: RETURN to home [%.2f %.2f] (%.1f m away), cruise %.2f m/s",
				 (double)_home(0), (double)_home(1),
				 (double)matrix::Vector2f(_home(0) - tick.pos.x, _home(1) - tick.pos.y).norm(),
				 (double)_config.fwd_spd);

		} else {
			// 没有有效的 home（没解锁过 / home_position 没来）就地降落，别乱飞。
			PX4_WARN("my_mission: no valid home position, landing here");
			enterState(State::Land, tick.now);
			_land_cmd_sent = false;
		}
	}
}

void MyMission::stateReturn(const Tick &tick)
{
	if (!_return_started) {
		// ---- 第一步：原地把机头对准 home ----
		// 不先转的话，回程是"侧着飞"——位置环照样能飞，但机头朝向和运动方向不一致，
		// 视觉上很怪，而且真机上摄像头/空速管的方向也不对。
		const matrix::Vector2f to_home(_home(0) - tick.pos.x, _home(1) - tick.pos.y);

		if (to_home.norm() < 1.0f) {
			// 已经在 home 正上方（或 1 m 以内），没有方向可言，直接开始
			_startReturnTransit(tick);
			return;
		}

		_yaw_sp = atan2f(to_home(1), to_home(0));
		_target = matrix::Vector3f(tick.pos.x, tick.pos.y, tick.pos.z);
		publishSetpoint(targetSetpoint(), tick.now);

		const float yaw_now = matrix::Eulerf(matrix::Quatf(tick.att.q)).psi();

		if (fabsf(matrix::wrap_pi(_yaw_sp - yaw_now)) < math::radians(5.f)) {
			_startReturnTransit(tick);

		} else if (elapsedSince(_state_start, tick.now) > _config.fwd_tmo) {
			// 转不过去（这个场景不该发生，但绝不能卡在 RETURN 里不出来）
			PX4_WARN("my_mission: yaw alignment timeout, landing here");
			enterState(State::Land, tick.now);
			_land_cmd_sent = false;
		}

		return;
	}

	if (advanceTransit(tick)) {
		_target = _goal3;
		enterState(State::Land, tick.now);
		_land_cmd_sent = false;
		PX4_INFO("my_mission: home reached [%.2f %.2f], LAND (mode %d)",
			 (double)_goal3(0), (double)_goal3(1), _config.land_mode);

	} else if (elapsedSince(_state_start, tick.now) > _config.fwd_tmo) {
		// 飞不回去（顶风 / 航向歪 / 位置估计漂）也不能一直在外面转：
		// 就地降落，降落本身是保底功能。
		PX4_WARN("my_mission: return timeout (traj %.2f m from home), landing here", (double)_goal_distance);
		enterState(State::Land, tick.now);
		_land_cmd_sent = false;
	}
}

void MyMission::_startReturnTransit(const Tick &tick)
{
	_goal3 = _home;

	// 和 enterTransit() 做同样的事，但**不能**调它：它会 enterState()，
	// 而那会把 _return_started 清掉，退回"对准航向"那一步。
	_transit.setLimits(transitLimits());
	_transit_acc_xy = 0.f;
	_transit_acc_along = 0.f;
	_decel_timer = 0.f;

	const matrix::Vector2f to_goal(_goal3(0) - tick.pos.x, _goal3(1) - tick.pos.y);
	_transit_dir = (to_goal.norm() > 0.5f) ? to_goal.unit()
		       : matrix::Vector2f(cosf(_yaw_sp), sinf(_yaw_sp));

	_transit.start(_goal3, tick.pos);
	_return_started = true;
	PX4_INFO("my_mission: heading home, %.1f m to go",
		 (double)(_goal3 - matrix::Vector3f(tick.pos.x, tick.pos.y, tick.pos.z)).norm());
}

void MyMission::stateLand(const Tick &tick)
{
	if (_config.land_mode == 1) {
		// ---- 方式 1：交给 PX4 的 AUTO_LAND ----
		// 发一次 MAV_CMD_NAV_LAND，commander 会强制切到 NAVIGATION_STATE_AUTO_LAND
		// （Commander.cpp:1236-1247，注意那里 force = true，所以随时可切）。
		// 之后的收油门 / flare / 着陆检测 / 按 COM_DISARM_LAND 自动上锁
		// 全部由 PX4 负责，比自己在 Offboard 里硬降稳得多。
		//
		// 只发一次：这是"切模式"命令，不是连续控制量，每拍重发没有意义。
		if (!_land_cmd_sent) {
			_output.publishCommand(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
			_land_cmd_sent = true;
		}

		// 这里**不再发 trajectory_setpoint**：nav_state 已经不是 Offboard，
		// 发了也没人看，留着只会让人误以为还在自己控。
		if (!tick.armed.armed) {
			// PX4 自己上锁了（COM_DISARM_LAND，默认 2 s），省掉我们的 DISARM 步骤
			_state = State::Done;
			PX4_INFO("my_mission: landed + disarmed");

		} else if (tick.land.landed && _config.auto_disarm) {
			// 已经判到着地，主动补一发 disarm（不等 PX4 那个 2 s）
			enterState(State::Disarm, tick.now);
			_last_disarm_cmd = 0;

		} else if (elapsedSince(_state_start, tick.now) > _config.land_tmo) {
			// 降不下去 / 着地检测一直不触发：退一步，直接尝试上锁并结束
			PX4_WARN("my_mission: land timeout, trying to disarm");
			enterState(State::Disarm, tick.now);
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
		float desc_rate = _config.desc_rate;
		const float min_vel = _external.inDescendMinVel();

		if (desc_rate < min_vel) {
			if (!_desc_rate_warned) {
				_desc_rate_warned = true;
				PX4_WARN("my_mission: MYM_DESC_RATE %.2f < %.2f (1.1*LNDMC_Z_VEL_MAX): raised, otherwise landing is undetectable",
					 (double)desc_rate, (double)min_vel);
			}

			desc_rate = min_vel;
		}

		if (tick.pos_valid) {
			_target(2) = math::min(_target(2) + desc_rate * tick.dt, _ground_z);
		}

		MissionOutput::Setpoint setpoint = targetSetpoint();
		setpoint.z_velocity = desc_rate;
		publishSetpoint(setpoint, tick.now);

		if (!tick.armed.armed) {
			_state = State::Done;

		} else if (tick.land.landed || tick.land.maybe_landed) {
			// **以 land detector 为准**收尾。
			// 不能用"目标 z 到了 + vz 小了"这种宽松判据：那只是"物理上贴地了"，
			// commander 的 disarm 检查要的是 land detector 的 landed/maybe_landed。
			// 我们一直在发 velocity[2] 前馈，所以 detector 会在触地后约 1 秒内
			// 依次满足 has_low_throttle -> ground_contact -> maybe_landed -> landed。
			// 兜底是 MYM_LAND_TMO（默认 40 s），超时会走上面那条超时分支。
			if (_config.auto_disarm) {
				enterState(State::Disarm, tick.now);
				_last_disarm_cmd = 0;
				PX4_INFO("my_mission: land detector confirmed (%s), disarming",
					 tick.land.landed ? "landed" : "maybe_landed");

			} else {
				_state = State::Done;
			}
		}
	}
}

void MyMission::stateDisarm(const Tick &tick)
{
	// MAV_CMD_COMPONENT_ARM_DISARM 的 param1：1 = arm，0 = disarm。
	// 这里 param1 = 0。param2 保持 0 = 普通上锁：PX4 会拒绝"空中上锁"，
	// 只有 param2 = 21196（force）才允许在空中强行断动力 —— 那是紧急手段，
	// 这里不用。
	if (!tick.armed.armed) {
		// ⚠ 这里**故意不刷 _state_start**（原实现就是这样）：Done 的接管检查窗口
		// 因此是从进 DISARM 那一刻算的，而正常落地时它早就过了 1 s，等于不检查。
		// 落地后本来也不会再掉高度，所以保持原样不动。
		_state = State::Done;
		PX4_INFO("my_mission: DISARMED");

	} else {
		// 1 Hz 重试：commander 可能因为"还没判到着地"拒绝一次，过一会再试。
		if (_last_disarm_cmd == 0 || elapsedSince(_last_disarm_cmd, tick.now) > 1.f) {
			_output.publishCommand(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);
			_last_disarm_cmd = tick.now;
		}

		if (elapsedSince(_state_start, tick.now) > _config.disarm_tmo) {
			PX4_WARN("my_mission: disarm timeout, giving up");
			_state = State::Done;
		}
	}
}

void MyMission::stateDone(const Tick &tick)
{
	// 任务结束。仍然把 setpoint 钉在当前位置：如果飞手这时又切进 Offboard，
	// 位置环的目标就是当前位置，不会飞回旧的悬停点。
	holdCurrentPosition(tick.pos, tick.att, tick.pos_valid);
	publishSetpoint(targetSetpoint(), tick.now);

	// 已上锁 → 回到 WAIT：下一次 arm + 切 Offboard 会自动重跑整个流程。
	if (!tick.armed.armed) {
		enterState(State::Wait, tick.now);
		_takeover_warned = false;   // 允许下一轮再告警
	}
}

// ===========================================================================
// 命令行
// ===========================================================================
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
		 _config.land_mode, (int)_config.auto_disarm, (double)_config.alt, (double)_config.hover);
	PX4_INFO("forward %.2f m  dir %.0f deg  tmo %.1f s  takeoff_yaw %.0f deg",
		 (double)_config.fwd_dist, (double)_config.fwd_dir_deg,
		 (double)_config.fwd_tmo, (double)math::degrees(_takeoff_yaw));

	// ---- 系绳测试 ----
	vehicle_local_position_s local_pos{};
	_local_pos_sub.copy(&local_pos);
	const matrix::Vector3f here(local_pos.x, local_pos.y, local_pos.z);

	PX4_INFO("rope pull: pitch_down %.1f deg, hold %.1f s, thr %.0f%% (entered when airborne)",
		 (double)_config.pitch_down_deg, (double)_config.tilt_hold,
		 (double)(_config.fwd_thr * 100.f));

	// 接管安全：现在把模式开关拨出 Offboard 会不会掉高度？
	manual_control_setpoint_s manual_control{};
	_manual_control_sub.copy(&manual_control);

	const bool centered = TakeoverMonitor::throttleCentered(manual_control, _external.manDeadzone());

	PX4_INFO("throttle %.2f -> %s (MAN_DEADZONE %.2f)  takeover_warned %d",
		 (double)manual_control.throttle,
		 centered ? "CENTERED (Position/Altitude will hold altitude)"
		 : "OFF-CENTER (Position/Altitude will descend immediately)",
		 (double)_external.manDeadzone(), (int)_takeover_warned);

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
  -> TILT (MYR_ROPE_EN = 1: go straight from the take-off point, pitch
           MYR_PITCH_DOWN nose down for MYR_TILT_HOLD seconds and add
           MYR_FWD_THR to the forward thrusters; no transit, no geometry)
     or FORWARD (MYR_ROPE_EN = 0 and MYM_FWD_DIST > 0: fly MYM_FWD_DIST along
           the take-off heading, offset by MYM_FWD_DIR)
  -> HOVER (MYM_HOVER)
  -> LAND (offboard descent or PX4 AUTO_LAND) -> DISARM -> DONE

MYR_ROPE_EN is a plain switch. The mission does not model the rope at all: the
rope itself lives in the simulator (SDF <rope_length>/<anchor_offset>/
<stiffness>, or GZ_TETHER_LEN), and the test is set up so that it is already
taut when the vehicle reaches MYM_ALT. All the mission does is hold the
take-off point and feed a horizontal acceleration forward.

TILT is realised as an acceleration feedforward of g*tan(MYR_PITCH_DOWN) along
the takeoff heading; the rope holds the vehicle back, so the resulting attitude
is the requested pitch with the position essentially unchanged.

Publishes offboard_control_mode{position=true} and trajectory_setpoint at 50 Hz,
and uses vehicle_command for landing/disarming.

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
