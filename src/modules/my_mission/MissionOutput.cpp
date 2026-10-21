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
 * @file MissionOutput.cpp
 */

#include "MissionOutput.hpp"

void MissionOutput::publishControlMode(hrt_abstime now)
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

void MissionOutput::publishSetpoint(const Setpoint &setpoint, hrt_abstime now)
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
	if (!_offboard) {
		return;
	}

	trajectory_setpoint_s tsp{};
	tsp.timestamp = now;

	// ---- 位置（NED，向上为负）----
	// 调用方已经解析好了：要么是 _target，要么是平滑器给出的轨迹位置，
	// 位置估计无效时给 NaN（= 不控制位置）。
	// 千万不要发 (0,0,0)：那会被理解成"飞到 EKF 原点"。
	tsp.position[0] = setpoint.position(0);
	tsp.position[1] = setpoint.position(1);
	tsp.position[2] = setpoint.position(2);

	// ---- 其余通道默认"不控制" ----
	// NaN 的语义是"这个状态不要你管"，位置环会自己规划出可行的速度/加速度。
	for (int i = 0; i < 3; i++) {
		tsp.velocity[i] = NAN;
		tsp.acceleration[i] = NAN;
		tsp.jerk[i] = NAN;
	}

	// ---- 加速度前馈（平滑器和低头段给）----
	// PositionControl.cpp:153 把它直接加进加速度指令（绕过速度环那个一阶滞后）。
	// 对平滑器来说这是修掉末端过冲的关键；对 TILT 来说这是低头角的来源。
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(setpoint.acceleration(i))) {
			tsp.acceleration[i] = setpoint.acceleration(i);
		}
	}

	// ---- 水平速度前馈（只有平滑器给）----
	// 位置环的结构是 vel_sp = 速度前馈 + MPC_XY_P * 位置误差（两者相加），
	// 前馈给成"setpoint 当前正在移动的速度"，位置误差就≈0，飞机精确跟着
	// setpoint 走：既不会因为追不上而滞后，也不会在停的时候冲过去。
	//
	// 只写 velocity[0] / velocity[1]，**不碰 velocity[2]** —— land detector 的
	// _in_descend 只读 velocity[2]，所以给水平前馈不会像"位置-only 的 NaN 速度"
	// 那样把着陆检测搞坏。
	if (PX4_ISFINITE(setpoint.velocity_xy(0)) && PX4_ISFINITE(setpoint.velocity_xy(1))) {
		tsp.velocity[0] = setpoint.velocity_xy(0);
		tsp.velocity[1] = setpoint.velocity_xy(1);
	}

	// ---- z 速度前馈（只有 Offboard 自己下降时给）----
	// PositionControl 的结构是 P-位置环 + 速度前馈（两者相加，不是二选一），
	// 所以给 velocity[2] 不会和 position[2] 打架。
	//
	// 自己下降时**必须**给：land detector 判断"是否被指令下降"读的就是它
	// （MulticopterLandDetector.cpp:248）。只发 position（velocity = NaN）会让
	// _in_descend 恒为 false，ground_contact 永远锁不上 -> landed 永远为 0
	// -> 后面发的 disarm 被 commander 拒。
	if (PX4_ISFINITE(setpoint.z_velocity)) {
		tsp.velocity[2] = setpoint.z_velocity;
	}

	// yaw 每拍都给锁存值（不然可能自己转）；yawspeed 给 NaN = 不控制偏航角速度。
	tsp.yaw = setpoint.yaw;
	tsp.yawspeed = NAN;

	_traj_sp_pub.publish(tsp);
}

void MissionOutput::publishCommand(uint16_t command, float param1, float param2)
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

void MissionOutput::publishAssist(bool enabled, hrt_abstime now)
{
	// 时间戳每拍刷新：控制器用 "now - timestamp < 500 ms" 判这条通道是否还活着，
	// 超时按"禁止"处理 —— 模块不发了就等于不管了，辅助退回关闭（fail-closed）。
	my_mission_assist_s assist{};
	assist.timestamp = now;
	assist.enabled = enabled;
	_assist_pub.publish(assist);
}
