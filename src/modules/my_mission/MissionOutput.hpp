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
	};

	MissionOutput() = default;
	~MissionOutput() = default;

	/// 每拍由状态机设置：只有真在 Offboard 里才允许发 trajectory_setpoint
	void setOffboard(bool offboard) { _offboard = offboard; }

	/// 50 Hz 一直调用（含 WAIT/DONE），否则 commander 会判 offboard signal lost
	void publishControlMode(hrt_abstime now);

	/// 位置 setpoint；内部会自己判断该不该发
	void publishSetpoint(const Setpoint &setpoint, hrt_abstime now);

	void publishCommand(uint16_t command, float param1 = 0.f, float param2 = 0.f);

	/// 前推力辅助的阶段许可。每拍一次；控制器在超时后按"允许"处理
	void publishAssist(bool enabled, hrt_abstime now);

private:
	bool _offboard{false};

	uORB::Publication<offboard_control_mode_s> _ocm_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _traj_sp_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<my_mission_assist_s> _assist_pub{ORB_ID(my_mission_assist)};
};
