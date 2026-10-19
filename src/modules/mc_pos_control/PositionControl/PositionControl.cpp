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

	// --- Forward-thrust assist ---------------------------------------------------------------
	// body_z is the direction of the body z axis in the local frame (it points down when level),
	// so a nose-down demand gives body_z(0) < 0. If the demanded pitch is steeper than the ceiling
	// we clamp the attitude there and turn the force deficit into forward thrust for the thrusters.
	//
	// The horizontal force of a tilted multirotor is (vertical force) * tan(pitch), so the deficit
	// is a tan() difference, not the sin() difference used by VtolType::pusher_assist(): the sine
	// version underestimates the missing force by ~18% at the operating points we care about
	// (24 deg demand vs 15 deg clamp: tan gives 7.30 N, sin gives 6.05 N).
	//
	// Keep the unlimited direction: MPC_TILTMAX_AIR may already have truncated the demand and the
	// deficit has to be measured against what was actually asked for, not against the truncation.
	const Vector3f body_z_raw = body_z;

	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	_fwd_thrust = 0.f;
	_fwd_acc_xy.setZero();

	// Effective nose-down ceiling: the tighter of MPC_PITCH_MIN and -MPC_TILTMAX_AIR, so the assist
	// can never loosen a limit that MPC_TILTMAX_AIR already imposed.
	const float pitch_ceiling = math::max(_pitch_min, -_lim_tilt);

	if (_fwd_thrust_en && (pitch_ceiling < 0.f) && (body_z_raw.norm_squared() > FLT_EPSILON)) {
		// _yaw_sp is only sanitised after _velocityControl() returns, so fall back to the state.
		const float yaw_ref = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
		const float cy = cosf(yaw_ref);
		const float sy = sinf(yaw_ref);

		// Rotate into the heading frame: the x component is the along-heading (pitch) part
		const float x_h = cy * body_z_raw(0) + sy * body_z_raw(1);   // along the heading
		const float z_h = body_z_raw(2);                             // downwards

		const float pitch = atan2f(x_h, z_h);                        // negative = nose down

		if (pitch < pitch_ceiling) {
			// Missing horizontal force: |tan(pitch)| - |tan(ceiling)|.
			// Both angles are negative, so negate to work with the magnitudes.
			const float deficit = tanf(-pitch) - tanf(-pitch_ceiling);
			_fwd_thrust = math::constrain(deficit * _fwd_thrust_sc, 0.f, _fwd_thrust_max);

			// Rebuild body_z with the pitch clamped, keeping roll and heading. In the heading frame
			// the body z axis is (sin(pitch)*cos(roll), -sin(roll), cos(pitch)*cos(roll)), unit
			// length. Take the roll from the already tilt-limited vector so the rebuild cannot
			// reintroduce tilt that MPC_TILTMAX_AIR just removed.
			const float y_lim = -sy * body_z(0) + cy * body_z(1);
			const float roll = -asinf(math::constrain(y_lim, -1.f, 1.f));
			const float cos_roll = cosf(roll);

			const float x_n = sinf(pitch_ceiling) * cos_roll;
			const float y_n = -sinf(roll);
			const float z_n = cosf(pitch_ceiling) * cos_roll;

			body_z(0) = cy * x_n - sy * y_n;
			body_z(1) = sy * x_n + cy * y_n;
			body_z(2) = z_n;

			// Safety net: roll plus a shallow pitch can still exceed the absolute tilt limit
			// (e.g. MPC_PITCH_MIN looser than MPC_TILTMAX_AIR), so re-apply it.
			ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

			// Horizontal acceleration the forward thrusters contribute. _thr_sp (below) only covers
			// the tilt, so without this the anti-windup in _velocityControl() sees a shortfall that
			// does not exist and freezes the velocity integrator: measured, a 15 m/s setpoint stalled
			// at 13.6 m/s against a phantom 1.26 m/s^2 gap.
			//
			// thrust_body[0] maps to a body-x force through K_F and SC = m*g/K_F, so g/SC is the
			// body-x acceleration per unit of thrust_body[0] -- no separate calibration needed.
			//
			// No tilt projection. a_body_x is by construction exactly the deficit this assist was
			// asked to fill (_fwd_thrust = (tan(pitch_demand) - tan(ceiling)) * SC, so
			// a_body_x = a_desired - a_tilt), which makes acc_sp_xy_produced equal a_desired whenever
			// the thrusters are not clipped -- which is the correct statement of "not saturated".
			// Projecting by cos(pitch) would instead under-count: the thrusters push along an axis
			// mounted ~20 deg above body x, so their true horizontal share at 15 deg pitch is
			// cos(pitch) + (AZ/AX)*sin(pitch) = 1.06, not 0.966. That phantom gap left a 0.07 m/s
			// residual (simulated 14.93 m/s against a 15 m/s setpoint).
			//
			// Once the assist clips at MYM_FWD_THR_MAX, a_body_x stops growing and
			// acc_sp_xy_produced falls below a_desired, so the anti-windup still engages exactly when
			// it should. The 0.342 vertical part of the thrust axis is left to the z integrator.
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

	// Forward thrust for the front-facing thrusters, computed in _accelerationControl().
	// thrust_body is normalised per axis by the allocator (ControlAllocationPseudoInverse divides
	// every thrust axis by the mean magnitude of its pseudo-inverse column), so this is NOT a force
	// in Newtons -- 1.0 means "the actuators that produce body-x force at full scale".
	// thrustToAttitude() only writes thrust_body[2] and the struct is zero initialised, so with
	// _fwd_thrust == 0 (assist disabled) the output is byte-identical to before this feature.
	attitude_setpoint.thrust_body[0] = _fwd_thrust;
}
