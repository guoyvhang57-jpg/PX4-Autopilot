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
 * 飞手要按什么顺序操作 —— 全在 dos/offboard.md，这里只讲结构。
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

	/**
	 * @brief 模块启动入口（`my_mission start`）
	 *
	 * 挂到工作队列上跑，所以只有一个任务，不存在多实例。
	 *
	 * @param argc 参数个数
	 * @param argv 参数表
	 * @return PX4_OK = 启动成功
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief 未识别的子命令
	 * @param argc 参数个数
	 * @param argv 参数表
	 * @return 永远返回错误，并打印用法
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @brief 打印模块用法的标题部分
	 * @param reason 打印前的提示语，可为 nullptr
	 * @return 永远返回错误码（用于 `my_mission help`）
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief `my_mission status`：打印状态机当前状态和几个关键量
	 *
	 * 包含状态、目标点、航向锁存值、拉绳段参数、以及"现在把模式拨出 Offboard
	 * 会不会掉高度"的提示。
	 *
	 * @return PX4_OK
	 */
	int print_status() override;

	/**
	 * @brief 初始化：拿参数、订阅 topic、把 Run() 挂到工作队列
	 * @return true = 成功
	 */
	bool init();

private:
	/**
	 * @brief 主循环，50 Hz
	 *
	 * 每拍取数 -> 刷参数 -> 发 offboard_control_mode / 辅助许可 -> 安全兜底检查
	 * -> 按状态分发表调用对应的 stateXxx()。
	 */
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

	/**
	 * @brief 状态名，用于日志和 `my_mission status`
	 * @param state 状态枚举
	 * @return 静态字符串，如 "FORWARD"
	 */
	static const char *stateName(State state);

	// ---- 9 个状态函数：每个都自己负责 _state / _state_start 的迁移 ----
	// 返回值故意用 void 而不是"下一个状态"：现有代码里有几处迁移顺带做了别的事
	// （_land_cmd_sent = false、_takeoff_yaw 锁存、几何解算……），而且 Disarm→Done
	// 是**故意不刷** _state_start 的。让每个函数照自己原来的写法改状态，
	// 这次重构才是"纯搬迁"，行为一个字节都不变。
	/**
	 * @brief 等待 armed + Offboard + 位置有效，并把 setpoint 钉在当前位置
	 *
	 * 三个条件齐了之后按"是不是已经离地"分叉：
	 * 在地上 -> TAKEOFF；已在空中且离 home 超过 MYM_ARRIVE -> TILT（拉绳测试）。
	 * 同时在这里锁存 home / 地面高度 / 起飞航向这三个基准。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateWait(const Tick &tick);

	/**
	 * @brief 垂直爬升到 home 上方 MYM_ALT
	 *
	 * 到位判据是"位置误差 < MYM_ARRIVE 且速度 < 0.3"。到位后按 MYM_FWD_DIST
	 * 决定进 FORWARD 还是直接 HOVER。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateTakeoff(const Tick &tick);

	/**
	 * @brief 用 TransitLeg 飞向"起飞点 + MYM_FWD_DIST"
	 *
	 * 每拍的轨迹推进、发布、到位判据都在 advanceTransit() 里；本函数只处理
	 * 到位之后的迁移（钉住终点 -> HOVER）和超时（MYM_FWD_TMO -> LAND）。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateForward(const Tick &tick);

	/**
	 * @brief 拉绳测试：机体坐标系下只推前后，左右和上下不动
	 *
	 * 沿机体 x 轴：位置和速度给定都用实测值，误差恒为 0，加速度指令里只剩
	 * 前馈 a_ff = g*tan(MYR_PITCH_DOWN)（MYR_FWD_THR > 0 时再加上推力桨的份额）。
	 * 横向和高度钉在进入 Offboard 的那个点，由位置环负责。保持 MYR_TILT_HOLD 秒，
	 * 时间到之后先把前馈斜坡退回 0 再转 LAND。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateTilt(const Tick &tick);

	/**
	 * @brief 保持当前 setpoint 悬停 MYM_HOVER 秒
	 *
	 * 之后有有效 home 就进 RETURN，没有就地降落（别乱飞）。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateHover(const Tick &tick);

	/**
	 * @brief 返航，分三个阶段
	 *
	 * ① 原地把机头对准 home（30 deg/s 斜坡 + 角速度前馈）；
	 * ② 用 TransitLeg 飞回 home；
	 * ③ 到了之后原地把航向转回起飞时的方向，然后降落。
	 *
	 * 每个阶段都有单独的超时兜底，转不过去 / 飞不回去就地降落。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateReturn(const Tick &tick);

	/**
	 * @brief RETURN 第一阶段结束：点亮回程轨迹
	 *
	 * 故意不复用 enterTransit()：那个会 enterState()，会把 _return_started 清掉，
	 * 于是又退回"对准航向"那一步。
	 *
	 * @param tick 这一拍的输入
	 */
	void _startReturnTransit(const Tick &tick);

	/**
	 * @brief 本拍该用多大 jerk —— 按需求姿态离低头限幅的比例在 max/min 之间插值
	 *
	 * 需求角取自**上一拍发出去的水平加速度前馈**（_transit_acc_xy），
	 * 只用 20 ms 之前的信息，不构成一个和位置环对抗的环。
	 *
	 * @return 本拍该用的 jerk 上限 [m/s^3]
	 */
	float scheduledJerk() const;

	/**
	 * @brief 航向是否已经稳定在目标上
	 *
	 * 两个条件都要满足：角度误差够小 **而且** 角速度也已经降下来 —— 只看误差的话
	 * 高速掠过目标那一刻也会被判成"到了"。
	 *
	 * @param tick       这一拍的输入（取实测姿态和角速度）
	 * @param target_yaw 目标航向 [rad]
	 * @return true = 误差 < kReturnYawErrMax 且角速度 < kReturnYawRateMax
	 */
	bool yawSettled(const Tick &tick, float target_yaw);

	/**
	 * @brief 让偏航给定以固定速率斜坡到目标，并把 setpoint 发出去
	 *
	 * 给定值自己按 kReturnYawSlew 走，同时把"正在以多快转"用 yawspeed 前馈给
	 * 姿态环的参考模型 —— 只给角度斜坡的话，误差永远只有一步，转不起来。
	 * 位置部分用当前的 _target，本函数只改航向。
	 *
	 * @param tick       这一拍的输入
	 * @param target_yaw 目标航向 [rad]
	 */
	void _rampYaw(const Tick &tick, float target_yaw);

	/**
	 * @brief 降落
	 *
	 * 方式 1（MYM_LAND_MODE=1）：发一次 NAV_LAND 交给 PX4 的 AUTO_LAND；
	 * 方式 0：自己在 Offboard 里按 MYM_DESC_RATE 把 z 目标一帧帧压到地面。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateLand(const Tick &tick);

	/**
	 * @brief 发上锁命令，1 Hz 重试直到真的退锁
	 *
	 * @param tick 这一拍的输入
	 */
	void stateDisarm(const Tick &tick);

	/**
	 * @brief 任务结束：把目标钉在当前位置等上锁
	 *
	 * 已上锁就回 WAIT，下一次 arm + Offboard 会自动重跑整个流程。
	 *
	 * @param tick 这一拍的输入
	 */
	void stateDone(const Tick &tick);

	/**
	 * @brief 切到下一个状态并刷新计时基准
	 *
	 * 顺带清掉那些"每段独立"的闩锁（减速闩锁、返航阶段标记、航向斜坡标记），
	 * 否则上一段的残留状态会带到下一段。
	 *
	 * @param next 下一个状态
	 * @param now  迁移发生的时刻
	 */
	void enterState(State next, hrt_abstime now)
	{
		_state = next;
		_state_start = now;
		_decel_latched = false;
		_decel_timer = 0.f;
		_return_started = false;
	}

	// ---- 中转（FORWARD / ROPE 共用）----
	/**
	 * @brief 从当前参数和 PX4 参数组装出 TransitLeg 的限幅
	 * @return 本次航段使用的限幅
	 */
	TransitLeg::Limits transitLimits() const;

	/**
	 * @brief 点亮中转轨迹（以 _goal3 为终点），并切到 next 状态
	 *
	 * @param tick 这一拍的输入
	 * @param next 中转结束后要进入的状态（Forward 或 Return）
	 */
	void enterTransit(const Tick &tick, State next);

	/**
	 * @brief 推进一拍中转、发布 setpoint，并判断是否到位
	 *
	 * 内部还负责：按需求姿态调度 jerk、判该不该关前推力辅助、给加速度前馈夹硬限。
	 *
	 * @param tick 这一拍的输入
	 * @return true = 轨迹贴住终点 **且** 飞机速度也够小
	 */
	bool advanceTransit(const Tick &tick);

	// ---- 系绳测试 ----
	/**
	 * @brief 打一行"这次中转要飞多远、用什么速度、估计多久"
	 *
	 * @param tick 这一拍的输入（起点取当前位置）
	 */
	void logTransitStart(const Tick &tick);

	// ---- 每拍 / 事件驱动的杂项 ----
	/// 把 MYM_* / MYR_* 的最新值刷进 _config 快照（启动时和每次 parameter_update）
	void refreshConfig();

	/**
	 * @brief 转发到 MissionOutput，声明位置型 Offboard
	 * @param now 本拍时间戳
	 */
	void publishControlMode(hrt_abstime now) { _output.publishControlMode(now); }

	/**
	 * @brief 转发到 MissionOutput，发布一帧 trajectory_setpoint
	 * @param setpoint 这一拍要发的内容
	 * @param now      本拍时间戳
	 */
	void publishSetpoint(const MissionOutput::Setpoint &setpoint, hrt_abstime now)
	{
		_output.publishSetpoint(setpoint, now);
	}

	/**
	 * @brief 什么都不额外给，只把位置钉在 _target 上
	 *
	 * 大多数状态用的就是这一帧：位置 = _target，速度/加速度为 NaN（不控制），
	 * 航向 = _yaw_sp。_target_valid 为 false 时位置发 NaN。
	 *
	 * @return 一帧只含位置和航向的 setpoint
	 */
	MissionOutput::Setpoint targetSetpoint() const;

	// ---- 状态判断 / 工具 ----
	/**
	 * @brief 位置估计现在能不能用
	 *
	 * xy_valid / z_valid 是 EKF 给的"这两个通道能不能信"；再加一个 500 ms 的
	 * 新鲜度判断 —— EKF 挂了之后 topic 还在，只看 valid 位会拿旧数据继续飞。
	 *
	 * @param pos 位置估计
	 * @param now 本拍时间戳
	 * @return true = 两个通道都有效且数据不超过 500 ms
	 */
	static bool localPositionValid(const vehicle_local_position_s &pos, hrt_abstime now);

	/**
	 * @brief 是否已经到达 _target
	 *
	 * 位置误差小于 MYM_ARRIVE **且** 速度小于 0.3 m/s —— 只判位置的话，飞过目标点
	 * 那一瞬间误差也是 0，会被误判成到了。
	 *
	 * @param pos 当前位置（含速度）
	 * @return true = 位置和速度都收住了
	 */
	bool reachedTarget(const vehicle_local_position_s &pos) const;

	/**
	 * @brief 把 _target 钉在当前位置、_yaw_sp 钉在当前航向
	 *
	 * WAIT / DONE 阶段这样发，飞机不会动；万一这时飞手切进 Offboard，位置环的
	 * 目标就是当前位置，不会突然冲出去。
	 *
	 * @param pos       当前位置
	 * @param att       当前姿态（取航向）
	 * @param pos_valid 位置是否有效；无效时不更新 _target
	 */
	void holdCurrentPosition(const vehicle_local_position_s &pos, const vehicle_attitude_s &att,
				 bool pos_valid);

	/**
	 * @brief 距某个时刻过去了多久
	 *
	 * @param start 起始时刻
	 * @param now   当前时刻
	 * @return 秒
	 */
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
	matrix::Vector3f _pull_origin{};  ///< 拉绳测试进入 Offboard 时记下的位置，TILT 的目标锚在这里
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
		(ParamFloat<px4::params::MYM_ALT_HIGH>) _param_mym_alt_high,
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
		(ParamFloat<px4::params::MYR_PULL_DRIFT>) _param_myr_pull_drift,
		(ParamFloat<px4::params::MYR_FWD_THR>) _param_myr_fwd_thr,
		(ParamFloat<px4::params::MYM_FWD_THR_SC>) _param_myr_fwd_thr_sc
	)
};
