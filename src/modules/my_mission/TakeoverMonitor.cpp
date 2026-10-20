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
 * @file TakeoverMonitor.cpp
 */

#include "TakeoverMonitor.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>

bool TakeoverMonitor::throttleCentered(const manual_control_setpoint_s &manual_control, float deadzone)
{
	return manual_control.valid && (fabsf(manual_control.throttle) <= deadzone);
}

bool TakeoverMonitor::evaluate(const vehicle_status_s &status, const manual_control_setpoint_s &manual_control,
			       float deadzone) const
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
		return false;
	}

	if (throttleCentered(manual_control, deadzone)) {
		return false;
	}

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
		(double)manual_control.throttle, (double)deadzone);

	return true;
}
