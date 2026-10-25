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
 * @file TakeoverMonitor.hpp
 *
 * 接管安全：飞手把模式开关拨出 Offboard 的那一刻，高度通道立刻由油门杆接管。
 * PX4 在 FlightTaskManualAltitude 里的行为是（每拍都跑，没有"等杆回中"的闩锁）：
 *
 *   _scaleSticks():
 *       vel_max_z = (throttle < 0) ? MPC_Z_VEL_MAX_DN : MPC_Z_VEL_MAX_UP;
 *       _velocity_setpoint(2) = vel_max_z * -throttle_expo;      // NED，正 = 下降
 *
 *   _updateAltitudeLock():
 *       apply_brake = fabsf(throttle_expo) <= FLT_EPSILON;
 *       杆在中位 -> _position_setpoint(2) = 当前位置（锁高度）
 *       杆不在中位 -> _position_setpoint(2) = NAN（速度环直接用上面那个速度）
 *
 * 所以"油门在最低"进 Position/Altitude = 立刻以 MPC_Z_VEL_MAX_DN（默认 1.5 m/s）
 * 下降。而手势解锁结束后油门正好就在最低 —— 这个坑是真实存在的。
 *
 * 本类只做"检测 + 告警"：模式是 commander 切的，模块拦不住，也不该去抢。
 */

#pragma once

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>

class TakeoverMonitor
{
public:
	TakeoverMonitor() = default;
	~TakeoverMonitor() = default;

	/**
	 * @brief 油门杆是不是在中位
	 *
	 * 和 PX4 判"油门在中位"逐字对齐：
	 *   FlightTaskManualAltitude::_updateAltitudeLock()
	 *     apply_brake = fabsf(_sticks.getThrottleZeroCenteredExpo()) <= FLT_EPSILON
	 *   Sticks::getThrottleZeroCenteredExpo() = expo(deadzone(throttle, MAN_DEADZONE), 0.6)
	 *   math::deadzone() 在 |x| <= deadzone 时返回 0，而 expo(0) = 0
	 *   => "在中位" 等价于 |throttle| <= MAN_DEADZONE（默认 0.1，归一化行程）
	 *
	 * @param manual_control 当前遥控器输入
	 * @param deadzone       油门死区（归一化行程），取自 MAN_DEADZONE
	 * @return true = 油门在中位，切 Position/Altitude 会保持高度
	 */
	static bool throttleCentered(const manual_control_setpoint_s &manual_control, float deadzone);

	/**
	 * @brief 当前这个模式离开 Offboard 会不会掉高度/掉高？
	 *
	 * 只有在"当前不在 Offboard、还没上锁、油门不在中位"时才算危险。判定通过时
	 * 会打一条 Critical 告警，但**拦不住**飞机 —— 模式是 commander 切的，
	 * 本模块不抢控制权，只负责让人知道接下来会发生什么。
	 *
	 * @param status         当前载具状态（取 nav_state 和 arming_state）
	 * @param manual_control 当前遥控器输入（取油门）
	 * @param deadzone       油门死区（归一化行程）
	 * @return true = 危险，并且已经打了一条告警（调用方自己决定要不要闩锁去重）
	 */
	bool evaluate(const vehicle_status_s &status, const manual_control_setpoint_s &manual_control,
		      float deadzone) const;
};
