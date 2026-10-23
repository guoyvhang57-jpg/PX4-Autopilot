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
 * @file TransitLeg.cpp
 */

#include "TransitLeg.hpp"

#include <mathlib/mathlib.h>

void TransitLeg::setLimits(const Limits &limits)
{
	// 每次进中转状态都设一遍（不是只在启动时设）：代价只是几个赋值，
	// 换来的是运行中改 MYM_* / MPC_* 立刻生效，不用重启模块。
	//
	// 注意 setCruiseSpeed 是**速度规划的上限**（_getMaxXYSpeed 里的
	// config.max_speed_xy）。忘了设的症状是"速度永远上不去"，而加速度和 jerk
	// 看起来都正常，很容易误判。
	const float spd = math::max(limits.cruise_speed, 0.01f);

	_smoothing.setCruiseSpeed(spd);
	_smoothing.setMaxAccelerationXY(math::max(limits.accel_xy, 0.05f));
	_smoothing.setMaxVelocityXY(math::max(limits.vel_xy_max, spd));
	_smoothing.setMaxJerk(math::max(limits.jerk, 0.1f));
	_smoothing.setMaxAllowedHorizontalError(math::max(limits.err_max_xy, 0.f));
	_smoothing.setMaxAllowedVerticalError(math::max(limits.err_max_z, 0.f));
	_smoothing.setMaxVelocityZ(math::max(limits.vel_z_max, 0.1f));
	_smoothing.setMaxAccelerationZ(math::max(limits.accel_z_max, 0.1f));
}

void TransitLeg::start(const matrix::Vector3f &goal, const vehicle_local_position_s &pos)
{
	_goal = goal;
	_goal_distance = (_goal - matrix::Vector3f(pos.x, pos.y, pos.z)).norm();

	// 不要拿实测加速度（pos.ax/ay/az）当轨迹初始状态。
	// 真机上这个量逐样本 std 有 1.4~2.2 m/s²，偶尔会抽到 5σ 的样本；平滑器
	// 会把它原样当成第一拍的加速度发出去 —— 实测抽到过 12.35 m/s²，超过 1 g，
	// 位置控制器的 body_z = (-a_x, -a_y, g - a_z) 里 g - a_z 变号，姿态解算
	// 退化，直接给出 30° 滚转（10-22 那趟的真实事故）。
	// 传 0：平滑器只用位置和速度重新规划，第一拍的加速度一定由它按
	// MPC_ACC_HOR / MPC_ACC_UP_MAX 自己算，不可能超限。
	_smoothing.reset(matrix::Vector3f(),
			 matrix::Vector3f(pos.vx, pos.vy, pos.vz),
			 matrix::Vector3f(pos.x, pos.y, pos.z));
}

TransitLeg::Update TransitLeg::update(const vehicle_local_position_s &pos, float dt)
{
	// 让 PX4 自己的轨迹生成器算这一拍该发什么。
	// 输入：飞机的**真实位置**（不是上一次的 setpoint）+ 三维终点。
	// 外部前馈速度给 0 —— 本模块没有别的参考源，参考完全由平滑器生成。
	// force_zero_velocity_setpoint = false：正常生成轨迹，不是"要求立刻停住"。
	//
	// 平滑器每拍做三件事（PositionSmoothing.cpp:285-350）：
	//   1) 把"位置目标"换算成"速度目标"（速度上限由剩余距离反解，见 TrajMath.hpp）
	//   2) 用上一拍解出的三段时长，从**上一拍记录的状态**推进 dt
	//   3) 以推进后的状态为新起点，重解一次三段时长
	// 第 2 步从记录状态求值而不是增量累加，是它不漂的原因。
	PositionSmoothing::PositionSmoothingSetpoints sp;
	_smoothing.generateSetpoints(matrix::Vector3f(pos.x, pos.y, pos.z),
				     _goal,
				     matrix::Vector3f(),
				     dt, false, sp);

	Update u;
	u.position = sp.position;
	u.velocity_xy = matrix::Vector2f(sp.velocity(0), sp.velocity(1));
	u.acceleration = sp.acceleration;

	// 到位判据的口径用 MYM_ARRIVE（默认 0.20 m），比 PX4 任务航点的
	// NAV_ACC_RAD（10 m）严格得多 —— 我们要的是"停在这个点上"，不是"进圈算过"。
	_goal_distance = (_goal - sp.position).norm();
	u.goal_distance = _goal_distance;

	return u;
}

float TransitLeg::estimateDuration(float distance, float speed, float accel)
{
	const float spd = math::max(speed, 0.01f);
	const float acc = math::max(accel, 0.05f);
	const float d_ramp = spd * spd / acc;   // 加速段 + 减速段一共走的距离

	return (distance > d_ramp) ? (2.f * spd / acc + (distance - d_ramp) / spd)
	       : (2.f * sqrtf(math::max(distance, 0.f) / acc));
}
