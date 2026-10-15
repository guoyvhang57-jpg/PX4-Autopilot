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

#pragma once

#include <math.h>

#include <matrix/matrix/math.hpp>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

/**
 * @class MyMission
 *
 * Offboard 位置控制的最小任务示例（起飞 -> 向前飞到目标点 -> 悬停 -> 降落 -> 上锁）。
 *
 * ===========================================================================
 * Offboard 的三条 topic（发布逻辑的全文在 MyMission.cpp 顶部注释里）
 * ===========================================================================
 *   - offboard_control_mode{position=true}，**50 Hz 一直发**
 *       commander 靠它判断 "Offboard 可用"：至少一个 flag 为 true，且 timestamp
 *       距今 < COM_OF_LOSS_T（默认 1 s）。必须在切进 Offboard **之前**就开始发，
 *       否则模式开关切不进去；进去之后断了就触发 failsafe。
 *       position=true → SetpointType::Trajectory → mc_pos_control 接管位置环。
 *
 *   - trajectory_setpoint（NED：x 北 / y 东 / **z 向下**；velocity/acceleration/jerk
 *       给 NaN = 该通道不受控）。这是喂给位置环的目标位置。
 *
 *   - vehicle_command（内部命令，target_system/component = 0 广播）：
 *       MAV_CMD_NAV_LAND            → 交给 PX4 的 AUTO_LAND（降落方式 1）
 *       MAV_CMD_COMPONENT_ARM_DISARM → param1=0 上锁
 *
 * ===========================================================================
 * 注意：offboard_control_mode 是单实例 topic
 * ===========================================================================
 * 同一时刻只能有一个模块在发。towed_control 也在发（attitude=true），
 * 两个同时跑会互相覆盖，commander 只认最后写的那份。所以 airframe 里只自启一个。
 *
 * ===========================================================================
 * 类的形状（三个基类，顺序别改）
 * ===========================================================================
 *   ModuleBase             —— 命令行框架：`my_mission start/stop/status` 由它分发
 *   ModuleParams           —— 参数框架：DEFINE_PARAMETERS 里的 MYM_* 由它托管
 *   px4::ScheduledWorkItem —— 周期任务：ScheduleOnInterval() 定时，到点回调 Run()
 */
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

	// 状态机：整个任务就是这 7 个状态之间的迁移，迁移条件全在 Run() 里。
	// 前 5 个状态每拍都要发 offboard_control_mode（否则 Offboard 信号会超时），
	// 其中 Wait/Takeoff/Forward/Hover/Land(方式0) 还要发 trajectory_setpoint。
	enum class State {
		Wait,     ///< 等 armed + Offboard + 位置有效；setpoint 钉在当前位置
		Takeoff,  ///< 目标 = 地面参考上方 MYM_ALT，等位置/速度都到位
		Forward,  ///< 把 setpoint 以 MYM_FWD_SPD 推到前方点（MYM_FWD_ACC 限加減速）；仅 DIST>0 时进
		Hover,    ///< 保持同一 setpoint MYM_HOVER 秒
		Land,     ///< 方式 1：发 NAV_LAND 交 AUTO_LAND；方式 0：自己把 z 斜坡拉回地面
		Disarm,   ///< 发 COMPONENT_ARM_DISARM(param1=0)，1 Hz 重试直到 !armed
		Done      ///< 结束；已上锁就回 Wait，允许再跑一遍
	};

	static const char *stateName(State state);

	// ---- 发布：offboard 的三条 topic ----
	void publishOffboardControlMode(hrt_abstime now);   // 每拍都发，问"交给哪一级"
	// z_velocity：可选的"下降速度前馈"。PositionControl 是 P-位置环 + 速度前馈
	// （两者相加，不冲突），而 land detector 靠它判断"是否被指令下降"，所以
	// 自己下降时**必须**给，否则 landed 永远上不去。NED，向下为正。
	//
	// xy_velocity：可选的"水平速度前馈"，只有 FORWARD 的斜坡用（其余状态给默认
	// NaN = 不控制，让位置环自己去算）。给它的意义：位置环的结构是
	// vel_sp = 速度前馈 + MPC_XY_P * 位置误差，前馈给成"setpoint 正在移动的速度"
	// 之后位置误差≈0，飞机就精确跟着 setpoint 走，不会因为追不上而滞后/超调。
	// 只写 velocity[0..1]，不碰 velocity[2]，所以和 land detector 无关。
	void publishTrajectorySetpoint(hrt_abstime now, float z_velocity = NAN,
				       const matrix::Vector2f &xy_velocity = matrix::Vector2f(NAN, NAN));
	void publishVehicleCommand(uint16_t command, float param1 = 0.f, float param2 = 0.f);

	// ---- 状态判断 / 工具 ----
	bool localPositionValid(const vehicle_local_position_s &pos, hrt_abstime now) const;
	bool reachedTarget(const vehicle_local_position_s &pos) const;
	void holdCurrentPosition(const vehicle_local_position_s &pos, const vehicle_attitude_s &att, bool pos_valid);
	float elapsedSince(hrt_abstime start, hrt_abstime now) const { return (now - start) * 1e-6f; }

	// ---- 接管安全：油门杆回中检测 ----
	// 飞手把模式开关拨出 Offboard 时，高度通道立刻交回给油门杆：
	//   杆在中位 -> 锁高度；杆不在中位 -> 按 MPC_Z_VEL_MAX_DN 立刻下降
	// 这里只做"检测 + 告警"，不阻止飞手接管（也没法阻止，模式是 commander 切的）。
	bool throttleStickCentered(const manual_control_setpoint_s &manual_control) const;
	void checkTakeoverSafety(const vehicle_status_s &status, const manual_control_setpoint_s &manual_control);
	// 刷新本模块需要"从别人的参数里抄"的两个阈值：MAN_DEADZONE、LNDMC_Z_VEL_MAX
	void updateExternalParams();

	State _state{State::Wait};
	hrt_abstime _state_start{0};        ///< 进入当前状态的时刻，用来算超时/悬停计时
	hrt_abstime _last_disarm_cmd{0};    ///< 上次发 disarm 的时刻，DISARM 状态里做 1 Hz 重试
	bool _land_cmd_sent{false};         ///< NAV_LAND 是"切模式"命令，只在进 LAND 时发一次
	bool _target_valid{false};          ///< false 时 trajectory_setpoint 的位置发 NaN
	matrix::Vector3f _target{};         ///< 位置环目标，机体系无关的本地 NED 坐标
	float _ground_z{0.f};               ///< 进入 Offboard 时的 z（NED），作为降落的目标地面高度
	float _yaw_sp{0.f};                 ///< 锁存的航向，每拍原样发给 trajectory_setpoint.yaw
	float _takeoff_yaw{0.f};            ///< 起飞那一刻的航向，Forward 用它算"机头正前方"

	// ---- FORWARD 的位置斜坡（xy 两轴）----
	// _target 是"这一拍真正发出去的目标位置"，FORWARD 里它每拍朝 _fwd_goal 挪 ff_velocity*dt。
	// 两者分开的意义：以前是把目标一次性跳到终点（阶跃 -> 位置环瞬间要最大速度 ->
	// 姿态猛压）；现在终点和"正在走的中间点"分开，保证任意时刻的速度都可控。
	//
	// 三个速度量，别混：
	//   _ramp_speed      目标点**上一拍**的移动速度，状态量，只给下一拍的加速限幅用
	//   setpoint_speed   梯形曲线给出的速度（只看目标点离终点多远，**不看飞机在哪**）
	//   ff_speed         这一拍**真正推进 _target 并发给 PX4** 的前馈速度
	//                    = setpoint_speed × track_scale（跟踪限幅）
	//
	// ⚠ ff_speed 只能用在"推进 _target / 发 velocity"上，**绝不能写回 _ramp_speed**。
	// 写回去会把限幅反馈进加速斜坡（乘系数 s 的稳态是 a·dt·s/(1-s)，s=0.8 时只有
	// 0.08 m/s，斜坡永远起不来）。详见 Run() 里 FORWARD 分支的注释。
	matrix::Vector2f _fwd_goal{};       ///< FORWARD 的终点（xy，NED），进 FORWARD 时定死
	float _ramp_speed{0.f};             ///< 目标点上一拍的移动速度（m/s），下一拍做加速限幅用

	// ---- 接管安全（油门回中检测）----	// 用 param_find 而不是 DEFINE_PARAMETERS：MAN_DEADZONE 归 manual_control 管，
	// 这里只是"读它来和 PX4 的油门中位判据保持一致"，不想把它拉进本模块的参数表。
	param_t _param_man_deadzone{PARAM_INVALID};
	float _man_deadzone{0.1f};          ///< 缓存的 MAN_DEADZONE，决定"算不算在中位"
	param_t _param_lndmc_z_vel_max{PARAM_INVALID};
	// MPC_XY_ERR_MAX：复用 PX4 轨迹生成器的"允许最大水平跟踪误差"做跟踪限幅。
	// 同样是"和 PX4 行为对齐"的外部阈值，所以也用 param_find 读，不进本模块参数表。
	param_t _param_mpc_xy_err_max{PARAM_INVALID};
	float _mpc_xy_err_max{2.0f};        ///< 缓存的 MPC_XY_ERR_MAX（默认 2.0 m），0 = 关掉限幅
	float _in_descend_min_vel{0.275f};  ///< 1.1 * LNDMC_Z_VEL_MAX：land detector 认"被指令下降"的最低速度
	bool _desc_rate_warned{false};      ///< MYM_DESC_RATE 被抬高过只告警一次
	bool _takeover_warned{false};       ///< 离开 Offboard 的告警只发一次，回 WAIT 时复位
	bool _in_offboard{false};           ///< 本拍 nav_state 是不是 OFFBOARD；决定要不要发 trajectory_setpoint

	// ===================================================================
	// uORB 句柄
	// ===================================================================
	// 订阅：Run() 里 copy()/update() 取最新一份
	//   既然 offboard 位置控制依赖 EKF，位置/姿态/解锁/模式/着地这几条是必须的
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};    // nav_state
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};             // armed / kill / termination
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)}; // 位置/速度/有效性
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};        // 只用来取当前航向
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)}; // landed 判据
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)}; // 接管安全：油门杆位置

	// 发布：offboard 的两条 + 一条命令
	uORB::Publication<offboard_control_mode_s> _ocm_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	// 参数默认值/含义见 my_mission_params.yaml；改完不用重启模块（Run 里 updateParams）
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MYM_ALT>) _param_mym_alt,
		(ParamFloat<px4::params::MYM_FWD_DIST>) _param_mym_fwd_dist,
		(ParamFloat<px4::params::MYM_FWD_DIR>) _param_mym_fwd_dir,
		(ParamFloat<px4::params::MYM_FWD_SPD>) _param_mym_fwd_spd,
		(ParamFloat<px4::params::MYM_FWD_ACC>) _param_mym_fwd_acc,
		(ParamFloat<px4::params::MYM_FWD_TMO>) _param_mym_fwd_tmo,
		(ParamFloat<px4::params::MYM_HOVER>) _param_mym_hover,
		(ParamInt<px4::params::MYM_LAND_MODE>) _param_mym_land_mode,
		(ParamFloat<px4::params::MYM_DESC_RATE>) _param_mym_desc_rate,
		(ParamFloat<px4::params::MYM_ARRIVE>) _param_mym_arrive,
		(ParamFloat<px4::params::MYM_TKO_TMO>) _param_mym_tko_tmo,
		(ParamFloat<px4::params::MYM_LAND_TMO>) _param_mym_land_tmo,
		(ParamInt<px4::params::MYM_AUTO_DISARM>) _param_mym_auto_disarm,
		(ParamFloat<px4::params::MYM_DISARM_TMO>) _param_mym_disarm_tmo
	)
};
