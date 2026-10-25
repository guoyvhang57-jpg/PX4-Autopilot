/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	if (PX4_ISFINITE(_acc_sp(2))) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
			       + CONSTANTS_ONE_G - _acc_sp(2);
	}

}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

	// _thr_sp only covers the tilt. The forward thrusters add horizontal acceleration that is not in
	// it, and leaving it out makes the anti-windup below see a shortfall that does not exist.
	acc_sp_xy_produced += _fwd_acc_xy;

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
		const float arw_gain = 2.f / _gain_vel_p(0);
		const Vector2f acc_sp_xy = _acc_sp.xy();

		vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
	}

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();

	// --- 前推力辅助 ---
	// body_z 是机体系 z 轴在本地系里的方向（水平时它指向下方），所以"低头"的需求表现为
	// body_z(0) < 0。需求俯仰比上限更陡时，就把姿态钉在上限，把缺掉的那部分水平力
	// 交给前向推力桨。
	//
	// 倾斜多旋翼的水平力是 (竖直力) * tan(俯仰)，所以缺额要用 tan 的差，不能照抄
	// VtolType::pusher_assist() 里的 sin 差 —— 在我们关心的工况上 sin 版会低估约 18%
	// （需求 24° / 上限 15°：tan 给 7.30 N，sin 只给 6.05 N）。
	//
	// 这里要先留一份**没被 MPC_TILTMAX_AIR 截断**的方向：缺额必须按"真正要求了多少"
	// 来算，不能按截断后的角度算。
	const Vector3f body_z_raw = body_z;

	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	_fwd_thrust = 0.f;
	_fwd_acc_xy.setZero();

	// 实际生效的低头上限：MYM_PITCH_MIN 和 -MPC_TILTMAX_AIR 里更严的那个。
	// 这样辅助**永远不可能放宽** MPC_TILTMAX_AIR 已经施加的限制。
	const float pitch_ceiling = math::max(_pitch_min, -_lim_tilt);

	if (_fwd_thrust_en && (pitch_ceiling < 0.f) && (body_z_raw.norm_squared() > FLT_EPSILON)) {
		// _yaw_sp 要等 _velocityControl() 返回后才做合法性检查，这里先退回状态估计的航向。
		const float yaw_ref = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
		const float cy = cosf(yaw_ref);
		const float sy = sinf(yaw_ref);

		// 转到航向坐标系：x 分量就是沿机头方向（俯仰）的那一部分
		const float x_h = cy * body_z_raw(0) + sy * body_z_raw(1);   // 沿机头
		const float z_h = body_z_raw(2);                             // 向下

		const float pitch = atan2f(x_h, z_h);                        // 负 = 低头

		if (pitch < pitch_ceiling) {
			// 缺掉的水平力 = |tan(需求角)| - |tan(上限角)|。
			// 两个角都是负数，取负号换成绝对值来算。
			const float deficit = tanf(-pitch) - tanf(-pitch_ceiling);
			_fwd_thrust = math::constrain(deficit * _fwd_thrust_sc, 0.f, _fwd_thrust_max);

			// 把俯仰钉在上限、保留滚转和航向，重建 body_z。在航向坐标系里机体 z 轴是
			// (sin(pitch)*cos(roll), -sin(roll), cos(pitch)*cos(roll))，单位长度。
			// 滚转要取自**已经被限幅过**的那个向量，否则重建会把 MPC_TILTMAX_AIR
			// 刚削掉的倾斜角又带回来。
			const float y_lim = -sy * body_z(0) + cy * body_z(1);
			const float roll = -asinf(math::constrain(y_lim, -1.f, 1.f));
			const float cos_roll = cosf(roll);

			const float x_n = sinf(pitch_ceiling) * cos_roll;
			const float y_n = -sinf(roll);
			const float z_n = cosf(pitch_ceiling) * cos_roll;

			body_z(0) = cy * x_n - sy * y_n;
			body_z(1) = sy * x_n + cy * y_n;
			body_z(2) = z_n;

			// 兜底：滚转叠加一个浅俯仰仍可能超过总的倾角上限
			// （比如 MYM_PITCH_MIN 比 MPC_TILTMAX_AIR 更宽松），所以再夹一次。
			ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

			// 前向推力桨贡献的水平加速度。下面的 _thr_sp 只包含**倾角**那一份，不加这一项的话
			// _velocityControl() 里的抗积分饱和会看到一个根本不存在的缺额，把速度积分器冻住：
			// 实测 15 m/s 的给定卡在 13.6 m/s，对着一个虚构的 1.26 m/s^2 缺口。
			//
			// thrust_body[0] 通过 K_F 和 SC = m*g/K_F 映射到机体系 x 方向的力，所以 g/SC 就是
			// 每单位 thrust_body[0] 对应的机体系 x 加速度 —— 不需要另外标定。
			//
			// **不要乘倾角投影。** a_body_x 按构造就等于这个辅助要补的缺额
			// （_fwd_thrust = (tan(需求角) - tan(上限角)) * SC，于是
			//   a_body_x = a_需求 - a_倾角），
			// 只要推力桨没被限幅，acc_sp_xy_produced 就正好等于 a_需求 —— 这才是
			// "没饱和"的正确表述。乘 cos(pitch) 反而会少算：推力桨的推力轴装在机体系 x 上方
			// 约 20°，15° 俯仰时它们真正的水平份额是
			// cos(pitch) + (AZ/AX)*sin(pitch) = 1.06，不是 0.966；
			// 那个虚构的缺口留下过 0.07 m/s 的稳态残差（仿真里 14.93 m/s 对 15 m/s 的给定）。
			//
			// 一旦辅助被 MYM_FWD_THR_MAX 限住，a_body_x 就不再增长，acc_sp_xy_produced 会掉到
			// a_需求以下，抗积分饱和该介入的时刻它就介入了。推力轴那 0.342 的竖直分量
			// 留给 z 通道的积分器处理。
			if (_fwd_thrust_sc > FLT_EPSILON) {
				const float a_body_x = _fwd_thrust * CONSTANTS_ONE_G / _fwd_thrust_sc;
				_fwd_acc_xy = Vector2f(cy, sy) * a_body_x;
			}
		}
	}

	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;

	// 前向推力桨的推力，数值在 _accelerationControl() 里算好。
	// thrust_body 由分配器**逐轴归一化**（ControlAllocationPseudoInverse 会把每个推力轴除以
	// 它对应伪逆列的平均模长），所以这里的 1.0 不是 1 N，而是"产生机体系 x 向力的那些执行器
	// 满量程"。thrustToAttitude() 只写 thrust_body[2]，结构体又是零初始化，所以
	// _fwd_thrust == 0（辅助关闭）时输出和加这个功能之前**逐字节一致**。
	attitude_setpoint.thrust_body[0] = _fwd_thrust;
}
