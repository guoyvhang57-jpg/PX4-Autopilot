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
 * @file RopeGeometry.cpp
 */

#include "RopeGeometry.hpp"

#include <mathlib/mathlib.h>

RopeSolution RopeGeometry::solve(const matrix::Vector3f &takeoff_pos, float takeoff_yaw) const
{
	RopeSolution solution{};

	if (!enabled()) {
		return solution;
	}

	// ---- 锚点 ----
	// MYR_ANCHOR_* 是"地面系绳点在无人机地面机体坐标系里的 xyz"，那个系是
	// **去掉倾斜的机体系**（只有航向），X 前 / Y 右 / Z 下。转到 NED：
	//   X_body = ( cos psi, sin psi)   机头方向
	//                                   （航向从北起顺时针为正，NED 的 x 指北、
	//                                     y 指东也正好是顺时针，两个角度系统重合）
	//   Y_body = (-sin psi, cos psi)   右侧
	//   Z_body = NED 的 +z             两者都是"向下为正"，不用换号
	const float c = cosf(takeoff_yaw);
	const float s = sinf(takeoff_yaw);
	const matrix::Vector3f &off = _config.anchor_offset_body;

	solution.anchor(0) = takeoff_pos(0) + off(0) * c - off(1) * s;
	solution.anchor(1) = takeoff_pos(1) + off(0) * s + off(1) * c;
	solution.anchor(2) = takeoff_pos(2) + off(2);

	// ---- 绷紧点 ----
	// 距离锚点水平 L*cos(θ)、高出 L*sin(θ)。这样斜距正好等于 L，
	// 绳和地面的夹角正好等于 θ。
	//
	// 水平方位取**起飞航向**（不是"锚点->飞机"的方位）：飞机是沿着机头方向
	// 飞出去的，绳也就沿着那个方向被拉直。MYR_ANCHOR_X/Y = 0（默认）时两种取法一样。
	const float length = math::max(_config.rope_length, kMinLength);
	const float angle = math::radians(math::constrain(_config.rope_angle_deg, kMinAngleDeg, kMaxAngleDeg));

	const float horizontal = length * cosf(angle);
	const float vertical = length * sinf(angle);

	solution.taut(0) = solution.anchor(0) + horizontal * c;
	solution.taut(1) = solution.anchor(1) + horizontal * s;
	solution.taut(2) = solution.anchor(2) - vertical;   // NED 向上为负

	return solution;
}

float RopeGeometry::slant(const matrix::Vector3f &anchor, const matrix::Vector3f &pos)
{
	return (pos - anchor).norm();
}

float RopeGeometry::angleDeg(const matrix::Vector3f &anchor, const matrix::Vector3f &pos)
{
	const float up = anchor(2) - pos(2);          // NED：> 0 表示飞机在锚点上方
	const float horizontal = matrix::Vector2f(pos(0) - anchor(0), pos(1) - anchor(1)).norm();

	return math::degrees(atan2f(up, horizontal));
}
