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
 * @file ExternalParams.hpp
 *
 * "本模块要从**别人的**参数里抄的阈值"集中在这里。
 *
 * 用 param_find 而不是 DEFINE_PARAMETERS 是刻意的：这些参数归别人管
 * （MPC_* 归 mc_pos_control，MAN_DEADZONE 归 manual_control，
 * LNDMC_Z_VEL_MAX 归 land detector），本模块只是读它们来**和 PX4 的行为对齐**，
 * 不想把它们拉进自己的参数表。
 *
 * 三个理由（和 PX4 自己的做法一致）：
 *   1) 语义完全一致 —— GotoControl 就是这么喂 PositionSmoothing 的限制
 *      （GotoControl.hpp:99-102）；
 *   2) PX4 默认值都是调过的，自己再发明一套没有依据；
 *   3) 抄 PX4 的口径，PX4 改默认值时本模块自动跟着变。
 */

#pragma once

#include <px4_platform_common/param.h>

class ExternalParams
{
public:
	ExternalParams() = default;
	~ExternalParams() = default;

	/// 启动时调一次，之后每次收到 parameter_update 再调
	void refresh();

	/// 判"油门算不算在中位"的死区，归一化行程
	float manDeadzone() const { return value(ManDeadzone); }

	/// land detector 认"被指令下降"的最低速度 = 1.1 * LNDMC_Z_VEL_MAX
	float inDescendMinVel() const { return 1.1f * value(LndmcZVelMax); }

	// PositionSmoothing 的限制（口径同 GotoControl.hpp:99-102）
	float mpcXyErrMax() const { return value(MpcXyErrMax); }
	float mpcZErrMax() const { return value(MpcZErrMax); }
	float mpcXyVelMax() const { return value(MpcXyVelMax); }
	float mpcZvAutoUp() const { return value(MpcZvAutoUp); }
	float mpcAccUpMax() const { return value(MpcAccUpMax); }

	/// TILT 段把 MYR_PITCH_DOWN 夹在它以内 [deg]
	float mpcTiltmaxAir() const { return value(MpcTiltmaxAir); }

	/// 前推力辅助的低头角上限 [deg]，负值 = 低头；0 = 没设。TILT 用它当作"姿态能到多低"的上界
	float mymPitchMin() const { return value(MymPitchMin); }

	/// 前推力辅助的"缺额 [N] -> 归一化推力"换算系数，TILT 用它把油门反解成加速度
	float mymFwdThrSc() const { return value(MymFwdThrSc); }

private:
	enum Index {
		ManDeadzone = 0,
		LndmcZVelMax,
		MpcXyErrMax,
		MpcZErrMax,
		MpcXyVelMax,
		MpcZvAutoUp,
		MpcAccUpMax,
		MpcTiltmaxAir,
		MymPitchMin,
		MymFwdThrSc,
		Count
	};

	struct Mirror {
		const char *name;
		param_t handle;
		float value;
	};

	float value(int index) const { return _mirrors[index].value; }

	Mirror _mirrors[Count] = {
		{"MAN_DEADZONE",     PARAM_INVALID, 0.1f},
		{"LNDMC_Z_VEL_MAX",  PARAM_INVALID, 0.25f},
		{"MPC_XY_ERR_MAX",   PARAM_INVALID, 2.0f},
		{"MPC_Z_ERR_MAX",    PARAM_INVALID, 1.0f},
		{"MPC_XY_VEL_MAX",   PARAM_INVALID, 12.0f},
		{"MPC_Z_V_AUTO_UP",  PARAM_INVALID, 3.0f},
		{"MPC_ACC_UP_MAX",   PARAM_INVALID, 4.0f},
		{"MPC_TILTMAX_AIR",  PARAM_INVALID, 45.0f},
		{"MYM_PITCH_MIN",    PARAM_INVALID, -15.0f},
		{"MYM_FWD_THR_SC",   PARAM_INVALID, 0.44f},
	};
};
