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

static constexpr hrt_abstime STATUS_PERIOD_US = 500000; ///< status rate while no transition happens (2 Hz)
static constexpr float LOST_MARGIN_S = 0.5f;           ///< keep GDH_T_LOST this far below EKF2_NOAID_TOUT

bool FlightTaskMyNewMode::activate(const trajectory_setpoint_s &last_setpoint)
{
	const bool ret = FlightTaskManualPosition::activate(last_setpoint);

	_loadParams();
	_ladder.reset(_time_stamp_activate);
	_hover_detector.reset();
	_xy_reset_counter_valid = false;
	_last_vel_sp_xy = Vector2f(NAN, NAN);
	_speed_limit = NAN;
	_last_status_publish = 0;

	// latest estimator status even if it was published before this task existed
	_estimator_status_flags_sub.copy(&_flags);
	_estimator_test_ratios_sub.copy(&_test_ratios);

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
	_loadParams(); // parameter members are refreshed by FlightModeManager::handleParameterUpdate()
	_updateEstimatorStatus();
	_updateHoverDetector();
	_updateHealth();

	// ManualAltitude::update() calls our _scaleSticks() and _updateSetpoints()
	const bool ret = FlightTaskManualPosition::update();

	// remember the commanded horizontal velocity for the next cycle's hover detector;
	// a locked position (finite position setpoint, zero velocity setpoint) is "no motion"
	_last_vel_sp_xy = Vector2f(_velocity_setpoint.xy());
	return ret;
}

void FlightTaskMyNewMode::_loadParams()
{
	gdh::LadderParams p{};
	p.tr_degraded = _param_gdh_tr_deg.get();
	p.t_reject_s = _param_gdh_t_rej.get();
	p.eph_degraded = _param_gdh_eph_deg.get();
	p.eph_failing = _param_gdh_eph_fail.get();
	p.eph_lost = _param_gdh_eph_lost.get();
	p.t_escalate_s = _param_gdh_t_esc.get();
	p.t_recover_s = _param_gdh_t_rec.get();

	// LOST must act before EKF2 declares the horizontal solution invalid
	const float noaid_s = static_cast<float>(_param_ekf2_noaid_tout.get()) * 1e-6f;
	p.t_lost_s = math::min(_param_gdh_t_lost.get(), math::max(noaid_s - LOST_MARGIN_S, 0.f));
	_ladder.setParams(p); // also sanitises ranges and the eph ordering

	gdh::HoverParams h{};
	h.acc_thr = _param_gdh_hov_acc.get();
	h.vel_thr = _param_gdh_hov_vel.get();
	h.t_confirm_s = _param_gdh_hov_t.get();
	h.los_window_s = _param_gdh_los_t.get();
	h.los_angle_min_rad = math::radians(_param_gdh_los_ang.get());
	_hover_detector.setParams(h);
}

void FlightTaskMyNewMode::_updateEstimatorStatus()
{
	// follow the estimator instance the system actually uses (multi-EKF); keep the last
	// flags until the new instance publishes, staleness is judged from their timestamp
	estimator_selector_status_s selector;

	if (_estimator_selector_status_sub.update(&selector)) {
		if (selector.primary_instance != _estimator_instance) {
			_estimator_instance = selector.primary_instance;
			_estimator_status_flags_sub.ChangeInstance(_estimator_instance);
			_estimator_test_ratios_sub.ChangeInstance(_estimator_instance);
			_estimator_status_flags_sub.copy(&_flags);
			_estimator_test_ratios_sub.copy(&_test_ratios);
		}
	}

	_estimator_status_flags_sub.update(&_flags);
	_estimator_test_ratios_sub.update(&_test_ratios);
}

void FlightTaskMyNewMode::_updateHoverDetector()
{
	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();

	gdh::HoverInputs in{};
	in.now_us = _time_stamp_current;
	in.dt_s = _deltatime;
	in.acc_n = lpos.ax;
	in.acc_e = lpos.ay;
	in.vel_n = lpos.vx;
	in.vel_e = lpos.vy;
	in.vel_valid = lpos.v_xy_valid;
	in.vel_aided = _flags.cs_gnss_vel || _flags.cs_ev_vel || _flags.cs_opt_flow;
	in.sp_n = _last_vel_sp_xy(0);
	in.sp_e = _last_vel_sp_xy(1);
	in.has_beacon = (_param_gdh_bcn_en.get() != 0) && lpos.xy_valid;
	in.rel_n = lpos.x - _param_gdh_bcn_n.get();
	in.rel_e = lpos.y - _param_gdh_bcn_e.get();
	_hover_detector.update(in);
}

void FlightTaskMyNewMode::_updateHealth()
{
	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();

	gdh::LadderInputs &in = _inputs;
	in.now_us = _time_stamp_current;
	in.estimator_age_s = (_flags.timestamp > 0 && _time_stamp_current >= _flags.timestamp)
			     ? static_cast<float>(_time_stamp_current - _flags.timestamp) * 1e-6f : NAN;
	in.xy_valid = lpos.xy_valid && lpos.v_xy_valid;
	in.eph = lpos.eph;
	in.pos_aiding = _flags.cs_gnss_pos || _flags.cs_ev_pos || _flags.cs_aux_gpos;
	in.vel_aiding = _flags.cs_gnss_vel || _flags.cs_ev_vel || _flags.cs_opt_flow || _flags.cs_wind_dead_reckoning;
	in.range_only_aiding = false; // EKF2 has no single-beacon range observation yet
	in.dead_reckoning = _flags.cs_inertial_dead_reckoning;
	in.los_static = _hover_detector.state().los_static;

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

	if (!vel_xy.isAllFinite()) {
		vel_xy.zero();
	}

	switch (_ladder.level()) {
	case gdh::Health::HEALTHY:
		_speed_limit = _param_mpc_vel_manual.get();
		break;

	case gdh::Health::DEGRADED: {
			const float scale = math::constrain(_param_gdh_vel_deg.get(), 0.f, 1.f);
			_speed_limit = _param_mpc_vel_manual.get() * scale;
			vel_xy *= scale;
			break;
		}

	case gdh::Health::FAILING:
		_speed_limit = math::constrain(_param_gdh_vel_fail.get(), 0.f, _param_mpc_vel_manual.get());

		if (vel_xy.norm() > _speed_limit) {
			vel_xy = (_speed_limit > 0.f) ? vel_xy.normalized() * _speed_limit : Vector2f(0.f, 0.f);
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
		// leaving LOST: no stale horizontal acceleration, the position lock re-engages when slow
		FlightTaskManualPosition::_updateSetpoints();
	}
}

void FlightTaskMyNewMode::_publishStatus(bool transition)
{
	if (!transition && (_time_stamp_current - _last_status_publish) < STATUS_PERIOD_US) {
		return;
	}

	const vehicle_local_position_s &lpos = _sub_vehicle_local_position.get();
	const gdh::HoverState &hov = _hover_detector.state();

	gps_denied_hold_status_s status{};
	status.health = static_cast<uint8_t>(_ladder.level());
	status.health_prev = static_cast<uint8_t>(_ladder.previous());
	status.candidate = static_cast<uint8_t>(_ladder.candidate());
	status.transitions = _ladder.transitions();
	status.time_in_level_us = _ladder.timeInLevelUs(_time_stamp_current);
	status.reasons = _ladder.reasons();
	status.hover = hov.hover;
	status.hover_verified = hov.verified;
	status.accel_xy_filt = hov.acc_filt;
	status.los_static = hov.los_static;
	status.los_rate = hov.los_rate;
	status.pos_aiding = _inputs.pos_aiding;
	status.vel_aiding = _inputs.vel_aiding;
	status.range_only_aiding = _inputs.range_only_aiding;
	status.pos_test_ratio = _inputs.pos_test_ratio;
	status.estimator_age_s = _inputs.estimator_age_s;
	status.eph = lpos.eph;
	status.dead_reckoning_s = _ladder.deadReckoningSeconds(_time_stamp_current);
	status.speed_limit = _speed_limit;
	status.timestamp = hrt_absolute_time();
	_status_pub.publish(status);

	_last_status_publish = _time_stamp_current;
}
