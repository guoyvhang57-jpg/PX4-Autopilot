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

	_accelerationControl(dt);

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

void PositionControl::_accelerationControl(const float dt)
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();

	// --- 前推力辅助：把低头角钉在上限 ---
	// body_z 是机体系 z 轴在本地系里的方向（水平时指向下方），所以"低头"的需求表现为
	// body_z(0) < 0。需求俯仰比上限更陡时，把姿态钉在上限，缺掉的那部分水平力交给前向推力桨。
	//
	// 这一段只负责"要不要钉、钉在哪"。补多少推力在函数末尾算（_thr_sp 出来之后），
	// 因为缺口要拿夹过之后真正形成的 _thr_sp 去反推。
	//
	// 先留一份**没被 limitTilt 截断**的方向：要不要钉必须按"真正要求了多少"来算。
	const Vector3f body_z_raw = body_z;

	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	_fwd_thrust = 0.f;
	_fwd_acc_xy.setZero();

	// 实际生效的低头上限：MYM_PITCH_MIN 和 -MPC_TILTMAX_AIR 里更严的那个。
	// 这样辅助**永远不可能放宽** MPC_TILTMAX_AIR 已经施加的限制。
	const float pitch_ceiling = math::max(_pitch_min, -_lim_tilt);

	// _yaw_sp 要等 update() 里才做合法性检查，这里先退回状态估计的航向。
	// 航向在"判断要不要钉"和"把比力投到机头"两处都要用，提到两个分支外面算。
	const float yaw_ref = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
	const float cy = cosf(yaw_ref);
	const float sy = sinf(yaw_ref);

	// 俯仰这一拍被钉在上限 = 辅助在工作。
	bool assist_active = false;

	if (_fwd_thrust_en && (pitch_ceiling < 0.f) && (body_z_raw.norm_squared() > FLT_EPSILON)) {
		// 转到航向坐标系：x 分量就是沿机头方向（俯仰）的那一部分
		const float x_h = cy * body_z_raw(0) + sy * body_z_raw(1);   // 沿机头
		const float z_h = body_z_raw(2);                             // 向下

		const float pitch = atan2f(x_h, z_h);                        // 负 = 低头

		if (pitch < pitch_ceiling) {
			assist_active = true;

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
		}
	}

	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;

	// --- 推力桨补多少：把期望三轴比力投到机体坐标系，取 b_x 分量 ---
	//   f        = (a_x, a_y, a_z - g)                期望三轴力（NED 比力，m/s^2）
	//   f_b      = R^T f                              R 由钉过的 body_z + 给定航向定
	//   f_b(0)   = cos(pitch)*f_x - sin(pitch)*f_z    （航向系里 b_x = (cos, 0, -sin)）
	//
	// 原来看起来"不需要投影"，只是因为 b_z 本来就是从 f 定出来的，f_b 退化成 (0,0,-|f|)，
	// x 分量恒为 0；姿态一旦被上限截住，b_x 分量就不再是 0 —— **它本身就是缺额**，
	// 不需要另外定义缺口，也不需要 tan 那一层换算。
	//
	// 竖直方向仍用上面的原公式：它让**竖直分量**精确等于需求。推力桨轴还有 0.342 的
	// 竖直分量，所以总力会偏大约 2%、几乎全在 +z（轻微上飘，竖直积分器会压回来）。
	// 要让总力精确就得把 f_b(2) 也用上，那需要 CA_ROTORn_AZ/AX 这套旋翼几何 —— 先不做。
	if (assist_active && (_fwd_thrust_sc > FLT_EPSILON)) {
		const float f_h_x = cy * _acc_sp(0) + sy * _acc_sp(1);   // 航向系水平分量
		const float f_h_z = _acc_sp(2) - CONSTANTS_ONE_G;        // 竖直分量（向下为正）
		const float body_pitch = atan2f(cy * body_z(0) + sy * body_z(1), body_z(2));
		const float f_b0 = cosf(body_pitch) * f_h_x - sinf(body_pitch) * f_h_z;

		const float thrust_target = math::constrain(f_b0 * _fwd_thrust_sc / CONSTANTS_ONE_G, 0.f, _fwd_thrust_max);

		// 上升斜坡：推进电机一步到位会有冲击 —— 实测推力桨从 0 跳到 50% 油门
		// 的那一拍，航向漂了 32°，加速度振动指标从 5 涨到 12~14。
		// **只限制上升**：辅助一关、或者需求消失，输出必须立刻归零，
		// 不然"关掉辅助"之后电机还会按斜坡推好几秒。目标中途变小也立刻跟随。
		if (_fwd_thrust_slew > FLT_EPSILON) {
			_fwd_thrust = math::min(thrust_target, _fwd_thrust_prev + _fwd_thrust_slew * dt);

		} else {
			_fwd_thrust = thrust_target;
		}

		// 抗积分饱和比的是 _acc_sp.xy()，那是**世界水平**的量；f_b0 是机体系 x 方向的量，
		// 两者差推力桨轴 20° 上仰带来的 1/K_h（K_h = cos + (AZ/AX)*sin ~= 1.06）。
		// produced 偏低比力偏低更危险：ARW 就是拦着积分器去修缺口的那一环，模型一旦偏低，
		// 缺口就再也修不掉，只能变成稳态速度偏差；而力偏几个点会被积分器自己修掉。
		// 所以这里报世界水平的缺额，按斜坡/限幅后的交付比例折算。
		//
		// 不报这一项的话，抗积分饱和会看到一个根本不存在的缺额，把速度积分器冻住：
		// 实测 15 m/s 的给定卡在 13.6 m/s，对着一个虚构的 1.26 m/s^2 缺口。
		const Vector2f a_xy_tilt = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
		const float a_res_h = (Vector2f(_acc_sp) - a_xy_tilt).dot(Vector2f(cy, sy));

		if (thrust_target > FLT_EPSILON) {
			_fwd_acc_xy = Vector2f(cy, sy) * (a_res_h * (_fwd_thrust / thrust_target));
		}
	}

	// 斜坡的起点存给下一拍。辅助没跑（关闭 / 没到上限）时这里是 0，
	// 所以每次重新投入都从 0 爬起来，不会残留上一次的推力。
	_fwd_thrust_prev = _fwd_thrust;
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

	// 前向推力桨的 x 分量，数值在 _accelerationControl() 里算好。
	// thrust_body 由分配器**逐轴归一化**（ControlAllocationPseudoInverse 会把每个推力轴除以
	// 它对应列的平均模长），所以这里的 1.0 不是 1 N，而是"产生机体系 x 向力的那些执行器满量程"。
	// thrustToAttitude() 只写 thrust_body[2]，结构体又是零初始化，所以
	// _fwd_thrust == 0（辅助关闭）时输出和原版**逐字节一致**。
	attitude_setpoint.thrust_body[0] = _fwd_thrust;
}
