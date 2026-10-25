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
 * @file MissionOutput.hpp
 *
 * 本模块**所有出站 topic** 集中在这里：offboard_control_mode、trajectory_setpoint、
 * vehicle_command。状态机只构造 Setpoint，不直接碰 uORB。
 *
 * 集中管理的理由（尤其是 trajectory_setpoint）见 MissionOutput.cpp 开头。
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/my_mission_assist.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>

class MissionOutput
{
public:
	/**
	 * @brief 一帧位置 setpoint
	 *
	 * NaN 的语义是"这个通道不受控"，和 TrajectorySetpoint.msg 一致。
	 * 用结构体而不是一堆"填 NAN = 不用"的位置参数，是为了让调用点能看出意图：
	 *
	 *   PositionOnly -> 只填 position
	 *   Transit      -> position + velocity_xy + acceleration（平滑器的完整轨迹）
	 *   Tilt         -> position + 只有水平分量的 acceleration（低头角）
	 */
	struct Setpoint {
		matrix::Vector3f position{NAN, NAN, NAN};
		matrix::Vector2f velocity_xy{NAN, NAN};
		matrix::Vector3f acceleration{NAN, NAN, NAN};
		float z_velocity{NAN};   ///< 向下为正；只有 Offboard 自己下降时给
		float yaw{0.f};
		float yawspeed{NAN};     ///< 偏航角速度前馈 [rad/s]；NaN = 不控制（只有返航掉头给）
	};

	MissionOutput() = default;
	~MissionOutput() = default;

	/**
	 * @brief 每拍由状态机告知"现在在不在 Offboard 里"
	 *
	 * 只有真在 Offboard 里才允许发 trajectory_setpoint：非 Offboard 时那个 topic
	 * 是飞行任务和 land detector 的通道，抢着发会破坏着陆检测（见 .cpp 开头）。
	 *
	 * @param offboard 本拍是否处于 Offboard 模式
	 */
	void setOffboard(bool offboard) { _offboard = offboard; }

	/**
	 * @brief 发布 offboard_control_mode，声明"本模块要用位置型 Offboard"
	 *
	 * 必须**每拍都调**（含 WAIT / DONE）：commander 靠
	 * `now - ocm.timestamp < COM_OF_LOSS_T` 判外部信号还在不在，停发就会被判成
	 * offboard signal lost。
	 *
	 * @param now 本拍时间戳
	 */
	void publishControlMode(hrt_abstime now);

	/**
	 * @brief 发布一帧 trajectory_setpoint
	 *
	 * 内部自己判该不该发（不在 Offboard 里直接返回）。NaN 的通道会被跳过，
	 * 语义是"这个状态不要你管"。
	 *
	 * @param setpoint 这一拍要发的内容
	 * @param now      本拍时间戳
	 */
	void publishSetpoint(const Setpoint &setpoint, hrt_abstime now);

	/**
	 * @brief 发一条 vehicle_command（只用于切降落和上锁）
	 *
	 * 这是"动作"不是"连续控制量"，调用方负责只发一次。
	 *
	 * @param command MAVLink 命令号，如 VEHICLE_CMD_NAV_LAND
	 * @param param1  命令参数 1（如 ARM_DISARM 的 0 = 上锁）
	 * @param param2  命令参数 2（21196 = 强制空中上锁，本模块不用）
	 */
	void publishCommand(uint16_t command, float param1 = 0.f, float param2 = 0.f);

	/**
	 * @brief 发布"本阶段允不允许用前推力辅助"
	 *
	 * 控制器每拍读一次，超过 100 ms 没收到就按"允许"处理 —— 所以模块挂掉时
	 * 辅助会保持原状，不会因为断流而突然失效。
	 *
	 * @param enabled       本阶段是否允许辅助出力
	 * @param fwd_thrust_sc 缺额 -> 归一化前推力的换算系数（0 = 用控制器参数里的值）
	 * @param now           本拍时间戳
	 */
	void publishAssist(bool enabled, float fwd_thrust_sc, hrt_abstime now);

private:
	bool _offboard{false};

	uORB::Publication<offboard_control_mode_s> _ocm_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<my_mission_assist_s> _assist_pub{ORB_ID(my_mission_assist)};
};
