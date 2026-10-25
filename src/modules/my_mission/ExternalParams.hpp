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

	/**
	 * @brief 把外部参数的最新值抄进本地镜像
	 *
	 * 启动时调一次，之后每次收到 parameter_update 再调一次 —— 所以改 MPC_* /
	 * MAN_DEADZONE 不用重启本模块。
	 */
	void refresh();

	/**
	 * @brief 判"油门算不算在中位"的死区
	 * @return MAN_DEADZONE（归一化行程，默认 0.1）
	 */
	float manDeadzone() const { return value(ManDeadzone); }

	/**
	 * @brief land detector 认"被指令下降"的最低下降速度
	 *
	 * Offboard 自己下降时必须把 z 速度前馈给到这个值以上，否则 detector 的
	 * `_in_descend` 恒为 false，landed 永远判不出来，后面的上锁全被拒。
	 *
	 * @return 1.1 * LNDMC_Z_VEL_MAX [m/s]
	 */
	float inDescendMinVel() const { return 1.1f * value(LndmcZVelMax); }

	// ---- PositionSmoothing 的限幅（口径同 GotoControl.hpp:99-102）----

	/// @return MPC_XY_ERR_MAX，水平允许误差 [m]
	float mpcXyErrMax() const { return value(MpcXyErrMax); }

	/// @return MPC_Z_ERR_MAX，垂直允许误差 [m]
	float mpcZErrMax() const { return value(MpcZErrMax); }

	/// @return MPC_XY_VEL_MAX，水平速度硬上限 [m/s]
	float mpcXyVelMax() const { return value(MpcXyVelMax); }

	/// @return MPC_Z_V_AUTO_UP，自动模式上升速度 [m/s]
	float mpcZvAutoUp() const { return value(MpcZvAutoUp); }

	/// @return MPC_ACC_UP_MAX，垂直加速上限 [m/s^2]
	float mpcAccUpMax() const { return value(MpcAccUpMax); }

	/**
	 * @brief 姿态角硬上限
	 * @return MPC_TILTMAX_AIR [deg]，TILT 段用它夹 MYR_PITCH_DOWN
	 */
	float mpcTiltmaxAir() const { return value(MpcTiltmaxAir); }

	/**
	 * @brief 前推力辅助的低头角上限
	 * @return MYM_PITCH_MIN [deg]，负值 = 低头，0 = 没设。TILT 用它当"姿态能到多低"的上界
	 */
	float mymPitchMin() const { return value(MymPitchMin); }

	/**
	 * @brief 前推力辅助的"力缺额 -> 归一化推力"换算系数
	 *
	 * TILT 段靠它把 MYR_FWD_THR 反解成加速度，和辅助正向用的必须是同一个值。
	 *
	 * @return MYM_FWD_THR_SC；填 0 表示自动算（见 MyMission::refreshConfig）
	 */
	float mymFwdThrSc() const { return value(MymFwdThrSc); }

	// ---- 自动算 MYM_FWD_THR_SC 需要的机体物理量 ----

	/// @return MPC_THR_HOVER，悬停时的总距占比
	float mpcThrHover() const { return value(MpcThrHover); }

	/// @return CA_ROTOR0_CT，单个升力桨满油门推力 [N]
	float caRotorCt() const { return value(CaRotorCt); }

	/// @return CA_ROTOR4_AX，推力桨推力轴在机体 x 上的分量
	float caThrusterAx() const { return value(CaThrusterAx); }

	/// @return CA_ROTOR4_AZ，推力桨推力轴在机体 z（向下）上的分量，通常是负值
	float caThrusterAz() const { return value(CaThrusterAz); }

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
		MpcThrHover,
		CaRotorCt,
		CaThrusterAx,
		CaThrusterAz,
		Count
	};

	struct Mirror {
		const char *name;
		param_t handle;
		float value;
	};

	/**
	 * @brief 取某个镜像参数的最新值
	 * @param index Index 枚举里的下标
	 * @return 参数值；还没 refresh 过时是表里写的默认值
	 */
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
		{"MPC_THR_HOVER",    PARAM_INVALID, 0.25f},
		{"CA_ROTOR0_CT",     PARAM_INVALID, 40.0f},
		{"CA_ROTOR4_AX",     PARAM_INVALID, 0.94f},
		{"CA_ROTOR4_AZ",     PARAM_INVALID, -0.342f},
	};
};
