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
 * @file MissionConfig.hpp
 *
 * 本模块**自己的**参数（MYM_* / MYR_*）的一份只读快照。
 *
 * 为什么要有这一层：
 *   - 子组件（TransitLeg / RopeGeometry）不想依赖 ModuleParams；
 *   - 每拍从 ParamFloat 里逐个 .get() 会有 20 次原子读，
 *     快照成 POD 之后状态机里全是普通结构体访问，读起来也干净；
 *   - 参数热更新天然成立 —— 主类在 parameter_update 时重新填一遍。
 *
 * 默认值必须和 my_mission_params.yaml 一致：这里的值只在
 * "参数还没被加载"那一瞬间起作用，但写成两个不同的数会非常难查。
 */

#pragma once

#include "RopeGeometry.hpp"

struct MissionConfig {
	// ---- MYM_*：任务流程 ----
	float alt{1.5f};          ///< MYM_ALT 起飞目标高度 [m]
	float fwd_dist{3.f};      ///< MYM_FWD_DIST 前飞距离 [m]，0 = 跳过 FORWARD
	float fwd_dir_deg{0.f};   ///< MYM_FWD_DIR 前飞方向相对机头的偏角 [deg]
	float fwd_spd{0.5f};      ///< MYM_FWD_SPD 中转巡航速度 [m/s]
	float fwd_acc{1.f};       ///< MYM_FWD_ACC 中转加减速度上限 [m/s^2]
	float fwd_tmo{20.f};      ///< MYM_FWD_TMO 中转超时 [s]（FORWARD 和 ROPE 共用）
	float hover{5.f};         ///< MYM_HOVER 悬停 / TILT 保持时间 [s]
	int land_mode{1};         ///< MYM_LAND_MODE 0 = Offboard 下降，1 = AUTO_LAND
	float desc_rate{0.4f};    ///< MYM_DESC_RATE Offboard 下降速率 [m/s]
	float arrive{0.2f};       ///< MYM_ARRIVE 到位判定半径 [m]
	float tko_tmo{20.f};      ///< MYM_TKO_TMO 起飞超时 [s]
	float land_tmo{40.f};     ///< MYM_LAND_TMO 降落超时 [s]
	bool auto_disarm{true};   ///< MYM_AUTO_DISARM 着地后自己发 disarm
	float disarm_tmo{10.f};   ///< MYM_DISARM_TMO 上锁超时 [s]

	// ---- MYR_*：系绳测试 ----
	RopeConfig rope{};           ///< MYR_ANCHOR_{X,Y,Z} / MYR_ROPE_LEN / MYR_ROPE_ANGLE
	float pitch_down_deg{15.f};  ///< MYR_PITCH_DOWN 绷紧点上的低头角 [deg]
	float tilt_hold{10.f};       ///< MYR_TILT_HOLD 绷紧点上低头保持多久 [s]
	float fwd_thr{0.f};         ///< MYR_FWD_THR 前推力电机归一化油门；> 0 时姿态钉在低头上限

	/// 是否启用系绳段（MYR_ROPE_LEN > 0）
	bool ropeEnabled() const { return rope.rope_length > 0.f; }
};
