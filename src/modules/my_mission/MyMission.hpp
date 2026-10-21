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
 * @file MyMission.hpp
 *
 * Offboard 位置控制的任务模块。Offboard 本身是什么、三条 topic 怎么配合、
 * 飞手要按什么顺序操作 —— 全在 docs/offboard.md，这里只讲结构。
 *
 * ===========================================================================
 * 这个模块被拆成 6 块（每块一个文件）
 * ===========================================================================
 *   MyMission.{hpp,cpp}   任务编排：状态机 + 9 个状态函数。就是"什么时候做什么"。
 *   MissionConfig.hpp     MYM_* / MYR_* 参数的一份只读快照（POD）。
 *   MissionOutput.{...}   所有出站 topic（offboard_control_mode / trajectory_setpoint /
 *                         vehicle_command）。**只有它能发 topic。**
 *   TransitLeg.{...}      "用 PX4 的轨迹生成器飞到一个定点"，FORWARD 和 ROPE 共用。
 *   ExternalParams.{...}  从别人的参数（MPC_* / MAN_DEADZONE / LNDMC_Z_VEL_MAX）抄阈值。
 *   TakeoverMonitor.{...} 接管安全：飞出 Offboard 时油门没回中会掉高度。
 *
 * 依赖方向是单向的：MyMission → 其它，其它之间互不依赖。
 *
 * ===========================================================================
 * 类的形状（三个基类，顺序别改）
 * ===========================================================================
 *   ModuleBase             —— 命令行框架：`my_mission start/stop/status` 由它分发
 *   ModuleParams           —— 参数框架：DEFINE_PARAMETERS 里的 MYM_x / MYR_x 由它托管
 *   px4::ScheduledWorkItem —— 周期任务：ScheduleOnInterval() 定时，到点回调 Run()
 */

#pragma once

#include <math.h>

#include <matrix/matrix/math.hpp>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "ExternalParams.hpp"
#include "MissionConfig.hpp"
#include "MissionOutput.hpp"
#include "TakeoverMonitor.hpp"
#include "TransitLeg.hpp"

using namespace time_literals;

class MyMission : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	MyMission();
	~MyMission() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:
	void Run() override;

	// 状态机：整个任务就是这 9 个状态之间的迁移。
	// 前 9 个状态每拍都要发 offboard_control_mode（否则 Offboard 信号会超时），
	// 其中除 Land(方式1) / Disarm 之外还要发 trajectory_setpoint。
	enum class State {
		Wait,     ///< 等 armed + Offboard + 位置有效；setpoint 钉在当前位置
		Takeoff,  ///< 目标 = 地面参考上方 MYM_ALT，等位置/速度都到位
		Forward,  ///< 用 TransitLeg 飞到前方点（MYM_FWD_DIST / MYM_FWD_DIR）
		Tilt,     ///< 拉绳：目标留在起飞点，按 MYR_PITCH_DOWN / MYR_FWD_THR 发前馈，保持 MYR_TILT_HOLD 秒
		Hover,    ///< 保持同一 setpoint MYM_HOVER 秒
		Return,   ///< 先把机头对准 home，再飞回去（复用 FORWARD 的逻辑）
		Land,     ///< 方式 1：发 NAV_LAND 交 AUTO_LAND；方式 0：自己把 z 斜坡拉回地面
		Disarm,   ///< 发 COMPONENT_ARM_DISARM(param1=0)，1 Hz 重试直到 !armed
		Done,     ///< 结束；已上锁就回 Wait，允许再跑一遍
		Count     ///< 状态个数，给分发表做 static_assert 用
	};

	/**
	 * @brief 一拍的输入
	 *
	 * 9 个状态函数都只吃这一个参数。传引用而不是拷贝：这些都是 Run() 里的局部变量，
	 * 生命周期覆盖整个分发过程。
	 */
	struct Tick {
		hrt_abstime now;
		float dt;
		const vehicle_local_position_s &pos;
		const vehicle_attitude_s &att;
		const actuator_armed_s &armed;
		const vehicle_land_detected_s &land;
		bool pos_valid;
		bool offboard;
	};

	using Handler = void (MyMission::*)(const Tick &);

	static const char *stateName(State state);

	// ---- 9 个状态函数：每个都自己负责 _state / _state_start 的迁移 ----
	// 返回值故意用 void 而不是"下一个状态"：现有代码里有几处迁移顺带做了别的事
	// （_land_cmd_sent = false、_takeoff_yaw 锁存、几何解算……），而且 Disarm→Done
	// 是**故意不刷** _state_start 的。让每个函数照自己原来的写法改状态，
	// 这次重构才是"纯搬迁"，行为一个字节都不变。
	void stateWait(const Tick &tick);
	void stateTakeoff(const Tick &tick);
	void stateForward(const Tick &tick);
	void stateTilt(const Tick &tick);
	void stateHover(const Tick &tick);
	void stateReturn(const Tick &tick);
	void _startReturnTransit(const Tick &tick);   ///< RETURN 里"对准航向之后"点亮轨迹

	/**
	 * @brief 本拍该用多大 jerk —— 按需求姿态离低头限幅的比例在 max/min 之间插值
	 *
	 * 需求角取自**上一拍发出去的水平加速度前馈**（_transit_acc_xy），
	 * 只用 20 ms 之前的信息，不构成一个和位置环对抗的环。
	 */
	float scheduledJerk() const;

	/// 航向是否已经稳定在 target_yaw 上：误差够小 **而且** 不再转
	bool yawSettled(const Tick &tick, float target_yaw);

	/// 让偏航给定以固定速率斜坡到 target_yaw，并把 setpoint 发出去（用当前 _target）
	void _rampYaw(const Tick &tick, float target_yaw);
	void stateLand(const Tick &tick);
	void stateDisarm(const Tick &tick);
	void stateDone(const Tick &tick);

	void enterState(State next, hrt_abstime now)
	{
		_state = next;
		_state_start = now;
		_decel_latched = false;
		_decel_timer = 0.f;
		_return_started = false;
	}

	// ---- 中转（FORWARD / ROPE 共用）----
	TransitLeg::Limits transitLimits() const;
	void enterTransit(const Tick &tick, State next);
	bool advanceTransit(const Tick &tick);   ///< 推进一拍并发布，返回"到位"

	// ---- 系绳测试 ----
	void logTransitStart(const Tick &tick);

	// ---- 每拍 / 事件驱动的杂项 ----
	void refreshConfig();
	void publishControlMode(hrt_abstime now) { _output.publishControlMode(now); }
	void publishSetpoint(const MissionOutput::Setpoint &setpoint, hrt_abstime now)
	{
		_output.publishSetpoint(setpoint, now);
	}

	/// "什么都不额外给，只把位置钉在 _target 上"—— 大多数状态用的就是这一帧
	MissionOutput::Setpoint targetSetpoint() const;

	// ---- 状态判断 / 工具 ----
	static bool localPositionValid(const vehicle_local_position_s &pos, hrt_abstime now);
	bool reachedTarget(const vehicle_local_position_s &pos) const;
	void holdCurrentPosition(const vehicle_local_position_s &pos, const vehicle_attitude_s &att, bool pos_valid);
	static float elapsedSince(hrt_abstime start, hrt_abstime now) { return (now - start) * 1e-6f; }

	// ---- 组件 ----
	MissionOutput _output;
	TransitLeg _transit;
	ExternalParams _external;
	TakeoverMonitor _takeover;

	// ---- 配置快照 / 解出来的几何 ----
	MissionConfig _config;            ///< 每拍从参数刷新的只读快照

	// ---- 任务记忆（9 个状态共享，所以留在主类）----
	State _state{State::Wait};
	hrt_abstime _state_start{0};      ///< 进入当前状态的时刻，用来算超时/悬停计时
	hrt_abstime _last_disarm_cmd{0};  ///< 上次发 disarm 的时刻，DISARM 状态里做 1 Hz 重试
	hrt_abstime _last_rope_log{0};    ///< TILT 里 1 Hz 打一次斜距/绳角，和 gz 遥测对表
	float _tilt_ff{0.f};              ///< TILT 当前实际发出的加速度前馈 [m/s^2]（斜坡后的值）
	hrt_abstime _last_run{0};         ///< 上一拍的时刻，用来算真实的 dt（平滑器靠 dt 推进）
	bool _land_cmd_sent{false};       ///< NAV_LAND 是"切模式"命令，只在进 LAND 时发一次
	bool _target_valid{false};        ///< false 时 trajectory_setpoint 的位置发 NaN
	matrix::Vector3f _target{};       ///< 位置环目标，机体系无关的本地 NED 坐标
	matrix::Vector3f _goal3{};        ///< 中转目标（三维 NED）
	matrix::Vector3f _home{};         ///< 返航目标（home 的水平位置 + 当前高度）
	bool _return_started{false};      ///< RETURN 里：航向已对准、轨迹已点亮
	bool _home_yaw_done{false};       ///< RETURN 里：已到家，正在把航向转回初始方向
	hrt_abstime _home_rot_start{0};   ///< 到家后开始转航向的时刻，给这一步单独的预算
	float _yaw_target{0.f};           ///< 斜坡的终点（返航两个掉头阶段各设一次）
	bool _yaw_ramp_started{false};    ///< 斜坡已从实测机头起步（每段重置一次）
	matrix::Vector2f _transit_dir{};  ///< 中转航向的水平单位向量，进中转时闩锁
	float _transit_acc_xy{0.f};       ///< 上一拍发出去的水平加速度前馈的模 [m/s^2]，jerk 调度用
	float _transit_acc_along{0.f};    ///< 上一拍沿航向的有符号加速度 [m/s^2]，判减速段用
	float _decel_timer{0.f};          ///< 沿航向加速度连续为负的累计时间 [s]，0.2 s 去抖
	bool _decel_latched{false};       ///< 本段已进入减速段（单向闩锁，换状态时清）
	float _goal_distance{0.f};        ///< 平滑器离终点还剩多远（到位判据 / 超时日志用）
	float _ground_z{0.f};             ///< 地面参考的 z（NED），在 WAIT 里落地时就锁存，作为降落目标
	bool _ground_z_latched{false};    ///< 只在**还没起飞**时锁存一次；手动进场时进 Offboard 已经在空中了
	float _yaw_sp{0.f};               ///< 锁存的航向，每拍原样发给 trajectory_setpoint.yaw
	float _takeoff_yaw{0.f};          ///< 起飞那一刻的航向，Forward 用它算"机头正前方"

	// ---- 只告警一次的闩锁 ----
	bool _desc_rate_warned{false};      ///< MYM_DESC_RATE 被抬高过只告警一次
	bool _pitch_clamped_warned{false};  ///< MYR_PITCH_DOWN 顶到倾角上限只告警一次
	bool _takeover_warned{false};       ///< 离开 Offboard 的告警只发一次，回 WAIT 时复位

	// ===================================================================
	// uORB 句柄
	// ===================================================================
	// 订阅：Run() 里 copy()/update() 取最新一份
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};    // nav_state
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};             // armed / kill / termination
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)}; // 位置/速度/有效性
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};        // 只用来取当前航向
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)}; // landed 判据
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};      // 解锁时设的 home，判断手动进场用
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)}; // RETURN 掉头判据用
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};       // 读控制器给的前推力，判辅助何时可以关
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)}; // 接管安全：油门杆位置

	// 参数默认值/含义见 my_mission_params.yaml；改完不用重启模块（Run 里 updateParams）
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MYM_ALT>) _param_mym_alt,
		(ParamFloat<px4::params::MYM_FWD_DIST>) _param_mym_fwd_dist,
		(ParamFloat<px4::params::MYM_FWD_DIR>) _param_mym_fwd_dir,
		(ParamFloat<px4::params::MYM_FWD_SPD>) _param_mym_fwd_spd,
		(ParamFloat<px4::params::MYM_FWD_ACC>) _param_mym_fwd_acc,
		(ParamFloat<px4::params::MYM_FWD_JERK_MAX>) _param_mym_fwd_jerk_max,
		(ParamFloat<px4::params::MYM_FWD_JERK_MIN>) _param_mym_fwd_jerk_min,
		(ParamFloat<px4::params::MYM_FWD_TMO>) _param_mym_fwd_tmo,
		(ParamFloat<px4::params::MYM_HOVER>) _param_mym_hover,
		(ParamInt<px4::params::MYM_LAND_MODE>) _param_mym_land_mode,
		(ParamFloat<px4::params::MYM_DESC_RATE>) _param_mym_desc_rate,
		(ParamFloat<px4::params::MYM_ARRIVE>) _param_mym_arrive,
		(ParamFloat<px4::params::MYM_TKO_TMO>) _param_mym_tko_tmo,
		(ParamFloat<px4::params::MYM_LAND_TMO>) _param_mym_land_tmo,
		(ParamInt<px4::params::MYM_AUTO_DISARM>) _param_mym_auto_disarm,
		(ParamFloat<px4::params::MYM_DISARM_TMO>) _param_mym_disarm_tmo,
		(ParamFloat<px4::params::MYR_PITCH_DOWN>) _param_myr_pitch_down,
		(ParamFloat<px4::params::MYR_TILT_HOLD>) _param_myr_tilt_hold,
		(ParamFloat<px4::params::MYR_FWD_THR>) _param_myr_fwd_thr
	)
};
