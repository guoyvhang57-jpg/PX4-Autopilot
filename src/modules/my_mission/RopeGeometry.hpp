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
 * @file RopeGeometry.hpp
 *
 * 系绳（拉绳测试）的几何。
 *
 * **纯计算**：不 include 任何 uORB topic、不碰参数框架、不持有模块状态。
 * 所有输入由 RopeConfig 传进来（它由 MYR_* 参数填），
 * 所以这段逻辑可以脱离飞控直接算一遍验证，不需要飞。
 *
 * 三样东西：
 *
 *   基准点  —— 由调用方给（NED）。my_mission 传的是 **EKF 本地原点 (0,0,0)**，
 *              不是起飞点：锚点在地面上是固定的，不能随飞机走
 *   锚点    —— 基准点 + 地面机体系(MYR_ANCHOR_X, Y, Z)
 *              地面机体系 = 去掉倾斜的机体系：X 前 / Y 右 / Z 下
 *              （和 NED 一样向下为正，所以 MYR_ANCHOR_Z 一般填 0）
 *   绷紧点  —— 锚点沿**起飞航向**水平 L*cos(θ)，再往上 L*sin(θ)
 *              L = MYR_ROPE_LEN，θ = MYR_ROPE_ANGLE
 *
 * 绷紧点的定义就是"绳子刚好拉直、但还没产生拉力"：到锚点的斜距正好 = L，
 * 而且绳和地面的夹角正好 = θ。再往前一点绳就开始出力。
 */

#pragma once

#include <matrix/matrix/math.hpp>

/**
 * @brief MYR_* 参数组成的一份配置
 *
 * 全部来自参数，没有任何硬编码的几何量（只有合法范围的下/上限）。
 */
struct RopeConfig {
	matrix::Vector3f anchor_offset_body{};  ///< 锚点相对基准点，地面机体系 前/右/下 [m]
	float rope_length{0.f};                 ///< 绳长 [m]，0 = 不做系绳段
	float rope_angle_deg{30.f};             ///< 绷紧时绳和地面的夹角 [deg]
};

/**
 * @brief 解出来的几何
 */
struct RopeSolution {
	matrix::Vector3f anchor{};     ///< 锚点（NED）
	matrix::Vector3f taut{};       ///< 绷紧点（NED）
};

/**
 * @class RopeGeometry
 *
 * 把 RopeConfig + (基准点, 起飞航向) 解成 RopeSolution。
 */
class RopeGeometry
{
public:
	// 合法范围。参数里的 min/max 和这里一致（见 my_mission_params.yaml）。
	static constexpr float kMinLength = 0.1f;   ///< 比这短的绳没有意义
	static constexpr float kMinAngleDeg = 1.f;
	static constexpr float kMaxAngleDeg = 89.f; ///< 90° 就是垂直，水平距离 0

	RopeGeometry() = default;
	~RopeGeometry() = default;

	void setConfig(const RopeConfig &config) { _config = config; }

	const RopeConfig &config() const { return _config; }

	/// 绳长 > 0 才算启用了系绳段
	bool enabled() const { return _config.rope_length > 0.f; }

	/**
	 * @brief 解出锚点和绷紧点
	 *
	 * 只应该在 enabled() 为真时调用；否则返回全零（调用方自己用
	 * MissionConfig::ropeEnabled() 判断要不要用这个结果）。
	 *
	 * @param reference_pos 基准点（NED）。my_mission 传本地原点 (0,0,0)
	 * @param takeoff_yaw   起飞那一刻的航向 [rad]，决定绷紧点的方位
	 */
	RopeSolution solve(const matrix::Vector3f &reference_pos, float takeoff_yaw) const;

	/// 飞机到锚点的斜距 [m]
	static float slant(const matrix::Vector3f &anchor, const matrix::Vector3f &pos);

	/// 绳和地面的夹角 [deg]（飞机在锚点上方为正；正上方 = 90°）
	static float angleDeg(const matrix::Vector3f &anchor, const matrix::Vector3f &pos);

private:
	RopeConfig _config{};
};
