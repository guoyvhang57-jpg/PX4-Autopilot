/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

ModuleBase::Descriptor MulticopterRateControl::desc{task_spawn, custom_command, print_usage};

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
			_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
			_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
			_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);

			// 偏航权限的分配。分配器被刻意蒙住了推力桨的偏航力臂（CA_ROTORn_PY = 0），
			// 于是它们的差动落在分配的**零空间**里：任何力矩给定都到不了那里。
			// 这里把偏航积分通过电机 trim 主题送过去，并从力矩给定里**减去同样多**，
			// 保证两份贡献加起来始终等于需求。
			//
			// 这一对桨只能吸收它余量允许的部分，装不下的仍然留在升力桨上。
			// 于是这个划分随油门**连续**变化，推力桨关掉或饱和时自动退回改动前的行为，
			// 不存在"切换点"可以搞错。
			actuator_motors_trim_s motors_trim{};
			const float yaw_diff_gain = _param_mc_yaw_diff_gain.get();

			if (yaw_diff_gain > FLT_EPSILON && _vehicle_control_mode.flag_control_offboard_enabled) {
				const int act1 = _param_mc_yaw_diff_act1.get();
				const int act2 = _param_mc_yaw_diff_act2.get();
				actuator_motors_s motors{};

				if (act1 >= 0 && act1 < static_cast<int>(actuator_motors_s::NUM_CONTROLS)
				    && act2 >= 0 && act2 < static_cast<int>(actuator_motors_s::NUM_CONTROLS)
				    && act1 != act2
				    && _actuator_motors_sub.copy(&motors)) {

					// 这一对桨在执行器单位下的余量。用两者的**均值**，不要用读回来的原始值：
					// 读回值里已经含着上一拍的 trim，所以 min(u1, 1-u2) 会拿差动去减它自己的余量，
					// 造成振荡 —— 实测 0.30 需求的 f = 0.5 时，每拍在 0.30 和 0.20 之间来回。
					// trim 是等幅反相的，取均值就是没加 trim 时的工作点。
					const float u_mean = 0.5f * (motors.control[act1] + motors.control[act2]);

					// 换算系数不是常数。开了 THR_MDL_FAC 之后执行器指令是**归一化转速**，
					// 推力按 u^2 走，所以"每单位差动换来多少偏航力矩"是这个东西的导数：
					//
					//   d(tau)/d(diff) = 4 * PY * AX * CT * u_mean
					//
					// 在本机型上就是 44.7 * u_mean：19% 油门时 8.5 N·m/单位，50% 时 22.3，
					// 100% 时 44.7。MC_YAW_DIFF_GAIN 是按 u_mean = 0.5 定义的，所以要乘 0.5/u_mean。
					//
					// 这一项算错的后果一点也不微妙：19% 油门用固定增益时，实际只交出去它减掉的
					// 38% 的力矩，偏航积分于是涨了 2.6 倍去补这个洞（实测 0.046 -> 0.111 还在涨，
					// 预测值 0.121）。分母那个下限是为了推力桨关掉时除法不发散 —— 那时余量本来就是 0，
					// 什么都不会交出去。
					const float gain_eff = yaw_diff_gain / math::max(2.f * u_mean, 0.02f);

					const float headroom = math::max(0.f, math::min(u_mean, 1.f - u_mean));
					const float max_torque = headroom / gain_eff;

					// 交给这一对桨的份额，按它当前油门的比例算。**这是"不用 slew limiter
					// 也能双向平滑交接"的关键**：推力桨转速爬升，份额跟着爬升；它们降下来，
					// 份额也退回去，没交出去的那部分仍然留在升力桨上，由后者的积分器自己吸收。
					//
					// kYawDiffFullU 以上是全额交接。低于它份额线性降到 0，于是等这一对桨慢到
					// 什么都交不出来的时候，已经没有东西要退回去了 —— 实测航段末尾那一下突兀的
					// 甩尾（0.5 s 内航向 +9.7°），就是积分里的陈货被一次性倒给升力桨造成的。
					//
					// 0.14 是八份日志里这一对桨的中位工作油门（90 分位 0.24），所以整个巡航段
					// 积分都是全额交出去的。这里取**平方**，让份额在底部掉得更快：0.07 时是 0.25，
					// 而线性版是 0.50 —— 也就是积分的陈货更早开始归还，升力桨有时间慢慢吸收，
					// 而不是一次性接住。线性版的实测甩尾是 4.3 / 5.2 / 7.6 / 5.1 度，
					// 全有全无版是 7.2 / 11.0 / 11.9 度。
					static constexpr float kYawDiffFullU = 0.14f;
					const float share = math::constrain(sq(u_mean / kYawDiffFullU), 0.f, 1.f);

					// 真正交给这一对桨的力矩：先夹到它交付得出的范围内，再乘油门份额。
					const float yaw_torque = math::constrain(rate_ctrl_status.yawspeed_integ,
								 -max_torque, max_torque) * share;
					const float diff = yaw_torque * gain_eff;

					torque_setpoint(2) -= yaw_torque;

					// 正的偏航力矩（FRD 里 = 机头右转）来自"抬左边的推力桨、压右边的"，
					// 也就是 act2 加、act1 减 —— 因为这一对桨分别装在 -PY 和 +PY 上。
					// 如果发现这个辅助是在**对抗**扰动而不是抵消它，就把
					// MC_YAW_DIFF_ACT1 和 MC_YAW_DIFF_ACT2 对调。
					motors_trim.trim[act1] = -diff;
					motors_trim.trim[act2] = diff;
				}
			}

			// 每拍都发，即使没启用 —— 否则上一拍的 trim 会留在分配器里不散
			motors_trim.timestamp = hrt_absolute_time();
			_actuator_motors_trim_pub.publish(motors_trim);

			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(MulticopterRateControl::desc, argc, argv);
}
