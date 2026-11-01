/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "FlightTaskMyNewMode.hpp"

#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace matrix;

static constexpr float STICK_CENTRED = 0.05f;   ///< |pitch/roll stick| below this counts as centred
static constexpr float HOVER_LPF_TAU_S = 0.5f;  ///< time constant of the horizontal-acceleration low-pass
static constexpr hrt_abstime STATUS_PERIOD_US = 500000; ///< status rate while no transition happens (2 Hz)

bool FlightTaskMyNewMode::activate(const trajectory_setpoint_s &last_setpoint)
{
	const bool ret = FlightTaskManualPosition::activate(last_setpoint);

	_loadLadderParams();
	_ladder.reset(_time_stamp_activate);
	_xy_reset_counter_valid = false;
	_acc_xy_lpf.zero();
	_unaccelerated_since = 0;
	_hover = false;
	_speed_limit = NAN;
	_last_status_publish = 0;

	return ret;
}

bool FlightTaskMyNewMode::updateInitialize()
{
	// Only the altitude-level requirements: losing xy is handled by the ladder (LOST),
	// which falls back to altitude-hold with stick tilt instead of failing the task.
	return FlightTaskManualAltitude::updateInitialize();
}

bool FlightTaskMyNewMode::update()
{
	_loadLadderParams(); // parameter members are refreshed by FlightModeManager::handleParameterUpdate()
	_updateEstimatorStatus();
	_updateHoverDetector();
	_updateHealth();

	// ManualAltitude::update() calls our _scaleSticks() and _updateSetpoints()
	return FlightTaskManualPosition::update();
}

void FlightTaskMyNewMode::_loadLadderParams()
{
	gdh::LadderParams p{};
	p.tr_degraded = _param_gdh_tr_deg.get();
	p.t_reject_s = _param_gdh_t_rej.get();
	p.eph_degraded = _param_gdh_eph_deg.get();
	p.eph_failing = _param_gdh_eph_fail.get();
	p.eph_lost = _param_gdh_eph_lost.get();
	p.t_lost_s = _param_gdh_t_lost.get();
	p.t_escalate_s = _param_gdh_t_esc.get();
	p.t_recover_s = _param_gdh_t_rec.get();
	_ladder.setParams(p);
}

void FlightTaskMyNewMode::_updateEstimatorStatus()
{
	// follow the estimator instance the system actually uses (multi-EKF)
	estimator_selector_status_s selector;

	if (_estimator_selector_status_sub.update(&selector)) {
		if (selector.primary_instance != _estimator_instance) {
			_estimator_instance = selector.primary_instance;
			_estimator_status_flags_sub.ChangeInstance(_estimator_instance);
			_estimator_test_ratios_sub.ChangeInstance(_estimator_instance);
			_flags = {};
			_test_ratios = {};
		}
	}

	_estimator_status_flags_sub.update(&_flags);
	_estimator_test_ratios_sub.update(&_test_ratios);
}

void FlightTaskMyNewMode::_updateHoverDetector()
{
	// Hover regime = unaccelerated horizontally (accelerometer-derived NED acceleration) with
	// centred sticks, held for GDH_HOV_T. An accelerometer measures specific force: it cannot
	// tell hover from constant-velocity flight, so this flags "unaccelerated", not "stationary".
	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();
	const Vector2f acc_xy(lpos.ax, lpos.ay);

	if (acc_xy.isAllFinite() && _deltatime > 0.f) {
		const float alpha = _deltatime / (HOVER_LPF_TAU_S + _deltatime);
		_acc_xy_lpf += alpha * (acc_xy - _acc_xy_lpf);
	}

	const bool sticks_centred = _sticks.isAvailable() && (_sticks.getPitchRoll().norm() < STICK_CENTRED);
	const bool unaccelerated = _acc_xy_lpf.norm() < _param_gdh_hov_acc.get();

	if (sticks_centred && unaccelerated) {
		if (_unaccelerated_since == 0) {
			_unaccelerated_since = _time_stamp_current;
		}

		_hover = (_time_stamp_current - _unaccelerated_since) >= static_cast<hrt_abstime>(_param_gdh_hov_t.get() * 1e6f);

	} else {
		_unaccelerated_since = 0;
		_hover = false;
	}
}

void FlightTaskMyNewMode::_updateHealth()
{
	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();

	gdh::LadderInputs &in = _inputs;
	in.now_us = _time_stamp_current;
	in.xy_valid = lpos.xy_valid && lpos.v_xy_valid;
	in.eph = lpos.eph;
	in.pos_aiding = _flags.cs_gnss_pos || _flags.cs_ev_pos || _flags.cs_aux_gpos;
	in.vel_aiding = _flags.cs_gnss_vel || _flags.cs_ev_vel || _flags.cs_opt_flow;
	in.range_only_aiding = false; // EKF2 has no single-beacon range source yet (report: future work)
	in.dead_reckoning = _flags.cs_inertial_dead_reckoning;
	in.hover = _hover;

	float ratio = 0.f;

	if (_flags.cs_gnss_pos) {
		ratio = math::max(ratio, math::max(_test_ratios.gps_hpos[0], _test_ratios.gps_hpos[1]));
	}

	if (_flags.cs_ev_pos) {
		ratio = math::max(ratio, math::max(_test_ratios.ev_hpos[0], _test_ratios.ev_hpos[1]));
	}

	in.pos_test_ratio = PX4_ISFINITE(ratio) ? ratio : 0.f;

	// an EKF position reset while in the mode can mean the filter jumped onto a spoofed source
	in.pos_reset = false;

	if (!_xy_reset_counter_valid) {
		_xy_reset_counter_seen = lpos.xy_reset_counter;
		_xy_reset_counter_valid = true;

	} else if (lpos.xy_reset_counter != _xy_reset_counter_seen) {
		_xy_reset_counter_seen = lpos.xy_reset_counter;
		in.pos_reset = true;
	}

	const bool changed = _ladder.update(in);

	if (changed) {
		const gdh::Health level = _ladder.level();
		const char *from = gdh::healthName(_ladder.previous());
		const char *to = gdh::healthName(level);

		if (level > _ladder.previous()) {
			PX4_WARN("GPS-denied hold: %s -> %s (reasons 0x%03x, eph %.1f m)", from, to,
				 (unsigned)_ladder.reasons(), (double)lpos.eph);

		} else {
			PX4_INFO("GPS-denied hold: %s -> %s (reasons 0x%03x, eph %.1f m)", from, to,
				 (unsigned)_ladder.reasons(), (double)lpos.eph);
		}
	}

	_publishStatus(changed);
}

void FlightTaskMyNewMode::_scaleSticks()
{
	FlightTaskManualPosition::_scaleSticks(); // sets _velocity_setpoint.xy() from the sticks

	Vector2f vel_xy(_velocity_setpoint.xy());

	switch (_ladder.level()) {
	case gdh::Health::HEALTHY:
		_speed_limit = _param_mpc_vel_manual.get();
		break;

	case gdh::Health::DEGRADED:
		_speed_limit = _param_mpc_vel_manual.get() * math::constrain(_param_gdh_vel_deg.get(), 0.f, 1.f);
		vel_xy *= math::constrain(_param_gdh_vel_deg.get(), 0.f, 1.f);
		break;

	case gdh::Health::FAILING:
		_speed_limit = math::min(_param_gdh_vel_fail.get(), _param_mpc_vel_manual.get());

		if (vel_xy.norm() > _speed_limit) {
			vel_xy = vel_xy.normalized() * _speed_limit;
		}

		break;

	case gdh::Health::LOST:
		_speed_limit = NAN; // not position-controlled; _updateSetpoints() switches to tilt
		break;
	}

	_velocity_setpoint.xy() = vel_xy;
}

void FlightTaskMyNewMode::_updateSetpoints()
{
	if (_ladder.level() == gdh::Health::LOST) {
		// Altitude-mode behaviour: yaw and z as usual, horizontal sticks command tilt (acceleration)
		FlightTaskManualAltitude::_updateSetpoints();
		_position_setpoint.xy() = Vector2f(NAN, NAN);
		_velocity_setpoint.xy() = Vector2f(NAN, NAN);

	} else {
		FlightTaskManualPosition::_updateSetpoints();
	}
}

void FlightTaskMyNewMode::_publishStatus(bool transition)
{
	if (!transition && (_time_stamp_current - _last_status_publish) < STATUS_PERIOD_US) {
		return;
	}

	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();

	gps_denied_hold_status_s status{};
	status.health = static_cast<uint8_t>(_ladder.level());
	status.health_prev = static_cast<uint8_t>(_ladder.previous());
	status.candidate = static_cast<uint8_t>(_ladder.candidate());
	status.transitions = _ladder.transitions();
	status.time_in_level_us = _ladder.timeInLevelUs(_time_stamp_current);
	status.reasons = _ladder.reasons();
	status.hover = _hover;
	status.accel_xy_filt = _acc_xy_lpf.norm();
	status.pos_aiding = _inputs.pos_aiding;
	status.vel_aiding = _inputs.vel_aiding;
	status.range_only_aiding = _inputs.range_only_aiding;
	status.pos_test_ratio = _inputs.pos_test_ratio;
	status.eph = lpos.eph;
	status.dead_reckoning_s = _ladder.deadReckoningSeconds(_time_stamp_current);
	status.speed_limit = _speed_limit;
	status.timestamp = hrt_absolute_time();
	_status_pub.publish(status);

	_last_status_publish = _time_stamp_current;
}
