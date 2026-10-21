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
 * @file TransitLeg.hpp
 *
 * "用 PX4 自己的轨迹生成器飞到一个定点" —— FORWARD 和 ROPE 两段共用这一个东西。
 *
 * 为什么复用 PositionSmoothing 而不是自己逐拍推：
 *   以前是逐拍把目标点朝终点挪，速度曲线上只有"加速限幅/巡航上限/刹车限幅"三个
 *   约束，**没有加速度前馈**。结果参考做匀速减速时飞机永远比参考快 a·τ
 *   （τ = 1/MPC_XY_VEL_P_ACC ≈ 0.56 s），3 m 前进段稳定过冲 0.39~0.45 m。
 *   现在交给 PositionSmoothing（和 GotoControl 用的是同一套）：
 *     * 每拍先按上一拍解出的三段时长推进轨迹，再以新状态为起点重解一次
 *     * 位置/速度/加速度/jerk 全部解析求得，终点速度、加速度严格为零
 *     * 刹车速度上限由 computeMaxSpeedFromDistance 给（TrajMath.hpp:61-71），
 *       它**扣掉了 jerk 建立减速度期间的滑行距离**，比 sqrt(2ad) 保守
 *     * 输出的 acceleration 直接当加速度前馈发出去（这是修掉过冲的关键）
 *
 * ⚠ 这个类**不发布任何 topic**。发布权留给 MissionOutput，因为
 *   trajectory_setpoint 是共享 topic，非 Offboard 时发它会把 land detector 搞坏
 *   （见 MissionOutput.cpp 开头的长注释）。"谁发 topic"集中在一处是刻意的。
 */

#pragma once

#include <lib/motion_planning/PositionSmoothing.hpp>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_local_position.h>

class TransitLeg
{
public:
	/**
	 * @brief 平滑器的限幅
	 *
	 * 映射原则：**能用 MYM_ 参数的就用 MYM_，气动/机体限制用 PX4 的** ——
	 * 口径和 GotoControl.hpp:99-102 完全一致。
	 */
	struct Limits {
		float cruise_speed{0.5f};   ///< 速度规划上限（MYM_FWD_SPD）
		float accel_xy{1.f};        ///< 水平加减速上限（MYM_FWD_ACC）
		float vel_xy_max{12.f};     ///< 水平速度硬上限（MPC_XY_VEL_MAX）
		float jerk{4.f};            ///< 进中转那一刻的 jerk（运行时由 setJerk 调度）
		float err_max_xy{2.f};      ///< MPC_XY_ERR_MAX
		float err_max_z{1.f};       ///< MPC_Z_ERR_MAX
		float vel_z_max{3.f};       ///< MPC_Z_V_AUTO_UP
		float accel_z_max{4.f};     ///< MPC_ACC_UP_MAX
	};

	/**
	 * @brief 一拍的结果
	 *
	 * 三个量一起发 —— 加速度那一项是修掉末端过冲的关键（见文件头）。
	 */
	struct Update {
		matrix::Vector3f position;      ///< 位置参考（NED）
		matrix::Vector2f velocity_xy;   ///< 水平速度前馈（NED）
		matrix::Vector3f acceleration;  ///< 加速度前馈（NED）
		float goal_distance{0.f};       ///< 轨迹离终点还剩多远 [m]
	};

	TransitLeg() = default;
	~TransitLeg() = default;

	void setLimits(const Limits &limits);

	/**
	 * @brief 只改 jerk 上限，其余限制不动
	 *
	 * 给"按需求姿态调度 jerk"用：离低头限幅远时给大值、贴着限幅时给小值。
	 * 平滑器每拍都用这个值重规划，所以中途改是安全的 —— 走过的状态不动，
	 * 只改"从现在开始还能怎么变"，加速度连续，改的只是斜率。
	 */
	void setJerk(float jerk) { _smoothing.setMaxJerk(math::max(jerk, 0.1f)); }

	/**
	 * @brief 从飞机的当前三阶状态点亮平滑器，开始一段新航程
	 *
	 * 给加速度是关键 —— 轨迹的加速度从飞机此刻的加速度出发，
	 * 任何一个阶次上都不出现跳变。GotoControl.cpp:71-75 是同样的做法。
	 */
	void start(const matrix::Vector3f &goal, const vehicle_local_position_s &pos);

	/// 每拍推进一次。调用前必须先 start()。
	Update update(const vehicle_local_position_s &pos, float dt);

	/// 到位判据只比**轨迹**；"飞机速度也小"是调用方的策略，不在这里
	bool arrived(float arrive_radius) const { return _goal_distance < arrive_radius; }

	/**
	 * @brief 粗略估计"走 distance 米、巡航 speed、加减速 accel"要多久
	 *
	 * 就是理想梯形曲线的时间：加速段 + 匀速段 + 减速段。
	 * 真实的轨迹因为 jerk 受限会更慢一点，所以这是个**乐观下界**，
	 * 拿来和 MYM_FWD_TMO 对一下、提醒参数设小了够用。
	 * 短距离时全程都在加减速，永远到不了 speed，走的是三角形的那个分支。
	 */
	static float estimateDuration(float distance, float speed, float accel);

private:
	PositionSmoothing _smoothing;
	matrix::Vector3f _goal{};
	float _goal_distance{0.f};
};
