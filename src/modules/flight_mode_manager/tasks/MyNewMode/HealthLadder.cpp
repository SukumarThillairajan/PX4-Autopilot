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

#include "HealthLadder.hpp"

#include <cmath>

namespace gdh
{

static constexpr float US_PER_S = 1e6f;

static inline bool elapsed(uint64_t now_us, uint64_t since_us, float seconds)
{
	return now_us >= since_us && static_cast<float>(now_us - since_us) >= seconds * US_PER_S;
}

static inline float clampf(float x, float lo, float hi)
{
	return std::isfinite(x) ? (x < lo ? lo : (x > hi ? hi : x)) : lo;
}

const char *healthName(Health h)
{
	switch (h) {
	case Health::HEALTHY: return "HEALTHY";

	case Health::DEGRADED: return "DEGRADED";

	case Health::FAILING: return "FAILING";

	case Health::LOST: return "LOST";
	}

	return "?";
}

void LadderParams::sanitize()
{
	tr_degraded = clampf(tr_degraded, 0.01f, 1.f);
	t_reject_s = clampf(t_reject_s, 0.f, 60.f);
	eph_degraded = clampf(eph_degraded, 0.1f, 1e4f);
	eph_failing = clampf(eph_failing, eph_degraded, 1e4f);
	eph_lost = clampf(eph_lost, eph_failing, 1e4f);
	t_lost_s = clampf(t_lost_s, 0.f, 60.f);
	t_escalate_s = clampf(t_escalate_s, 0.f, 60.f);
	t_recover_s = clampf(t_recover_s, 0.f, 600.f);
	t_reset_hold_s = clampf(t_reset_hold_s, 0.f, 600.f);
	t_stale_s = clampf(t_stale_s, 0.5f, 60.f);
}

void HealthLadder::setParams(const LadderParams &p)
{
	_p = p;
	_p.sanitize();
}

void HealthLadder::reset(uint64_t now_us)
{
	_level = Health::HEALTHY;
	_prev = Health::HEALTHY;
	_candidate = Health::HEALTHY;
	_reasons = 0;
	_transitions = 0;
	_initialized = false;
	_t_level_start = now_us;
	_worse_active = false;
	_better_active = false;
	_dr_active = false;
	_reject_active = false;
	_reset_seen = false;
}

float HealthLadder::deadReckoningSeconds(uint64_t now_us) const
{
	return (!_dr_active || now_us < _t_dr) ? 0.f : static_cast<float>(now_us - _t_dr) / US_PER_S;
}

Health HealthLadder::classify(const LadderInputs &in, uint16_t &reasons)
{
	reasons = 0;
	Health h = Health::HEALTHY;

	auto raise = [&](Health level, uint16_t reason) {
		reasons |= reason;

		if (level > h) {
			h = level;
		}
	};

	// Stale or missing estimator status: the aiding flags cannot be trusted, assume none.
	const bool stale = !std::isfinite(in.estimator_age_s) || in.estimator_age_s > _p.t_stale_s;
	const bool pos_aiding = in.pos_aiding && !stale;
	const bool vel_aiding = in.vel_aiding && !stale;
	const bool range_aiding = in.range_only_aiding && !stale;

	// A single range observes only its line of sight: acceptable while the LOS rotates,
	// rank-1 while it is static (hover, or motion straight towards / away from the beacon).
	const bool range_counts = range_aiding && !in.los_static;
	const bool any_aiding = pos_aiding || vel_aiding || range_aiding;
	const bool dead_reckoning = (in.dead_reckoning && !stale) || !any_aiding;

	// timers
	if (dead_reckoning) {
		if (!_dr_active) {
			_dr_active = true;
			_t_dr = in.now_us;
		}

	} else {
		_dr_active = false;
	}

	const bool rejecting = pos_aiding && (in.pos_test_ratio > 1.f);

	if (rejecting) {
		if (!_reject_active) {
			_reject_active = true;
			_t_reject = in.now_us;
		}

	} else {
		_reject_active = false;
	}

	if (in.pos_reset) {
		_reset_seen = true;
		_t_reset = in.now_us;
	}

	const bool eph_finite = std::isfinite(in.eph);

	// LOST
	if (!in.xy_valid) {
		raise(Health::LOST, XY_INVALID);
	}

	if (eph_finite && in.eph > _p.eph_lost) {
		raise(Health::LOST, EPH_HIGH);
	}

	if (_dr_active && elapsed(in.now_us, _t_dr, _p.t_lost_s)) {
		raise(Health::LOST, DR_TIMEOUT);
	}

	// FAILING
	if (stale) {
		raise(Health::FAILING, ESTIMATOR_STALE);
	}

	if (_dr_active) {
		raise(Health::FAILING, (in.dead_reckoning && !stale) ? DEAD_RECKONING : (NO_POS_AIDING | NO_VEL_AIDING));
	}

	if (eph_finite && in.eph > _p.eph_failing) {
		raise(Health::FAILING, EPH_HIGH);
	}

	if (_reject_active && elapsed(in.now_us, _t_reject, _p.t_reject_s)) {
		raise(Health::FAILING, REJECTING);
	}

	// DEGRADED
	if (!pos_aiding && !range_counts) {
		raise(Health::DEGRADED, range_aiding ? RANGE_RANK1 : NO_POS_AIDING);
	}

	if (!vel_aiding) {
		reasons |= NO_VEL_AIDING; // informative only: position aiding alone still bounds the drift
	}

	if (pos_aiding && in.pos_test_ratio > _p.tr_degraded) {
		raise(Health::DEGRADED, TEST_RATIO_HIGH);
	}

	if (eph_finite && in.eph > _p.eph_degraded) {
		raise(Health::DEGRADED, EPH_HIGH);
	}

	if (_reset_seen && !elapsed(in.now_us, _t_reset, _p.t_reset_hold_s)) {
		raise(Health::DEGRADED, POS_RESET);
	}

	return h;
}

bool HealthLadder::update(const LadderInputs &in)
{
	uint16_t reasons = 0;
	const Health c = classify(in, reasons);
	_reasons = reasons;
	_candidate = c;

	if (!_initialized) {
		_initialized = true;
		_level = c;
		_prev = c;
		_t_level_start = in.now_us;
		_worse_active = false;
		_better_active = false;
		return false;
	}

	if (c > _level) {
		// Escalation: the timer runs while the candidate stays worse than the level, whatever
		// worse level it is, so a candidate flickering between two worse levels still escalates.
		_better_active = false;

		if (!_worse_active) {
			_worse_active = true;
			_t_worse = in.now_us;
		}

		const bool immediate = (reasons & XY_INVALID) != 0;

		if (immediate || elapsed(in.now_us, _t_worse, _p.t_escalate_s)) {
			_prev = _level;
			_level = c;
			_t_level_start = in.now_us;
			_worse_active = false;
			++_transitions;
			return true;
		}

	} else if (c < _level) {
		// Recovery: one level per t_recover_s of continuously better candidates.
		_worse_active = false;

		if (!_better_active) {
			_better_active = true;
			_t_better = in.now_us;
		}

		if (elapsed(in.now_us, _t_better, _p.t_recover_s)) {
			_prev = _level;
			_level = static_cast<Health>(static_cast<uint8_t>(_level) - 1);
			_t_level_start = in.now_us;
			_t_better = in.now_us; // the next step needs its own full period
			++_transitions;
			return true;
		}

	} else {
		_worse_active = false;
		_better_active = false;
	}

	return false;
}

void HoverDetector::reset()
{
	_s = HoverState{};
	_lpf_n = 0.f;
	_lpf_e = 0.f;
	_raw_active = false;
}

const HoverState &HoverDetector::update(const HoverInputs &in)
{
	// accelerometer-derived horizontal acceleration, low-passed
	if (std::isfinite(in.acc_n) && std::isfinite(in.acc_e) && in.dt_s > 0.f) {
		const float alpha = in.dt_s / (_p.lpf_tau_s + in.dt_s);
		_lpf_n += alpha * (in.acc_n - _lpf_n);
		_lpf_e += alpha * (in.acc_e - _lpf_e);
	}

	_s.acc_filt = std::sqrt(_lpf_n * _lpf_n + _lpf_e * _lpf_e);
	_s.unaccelerated = _s.acc_filt < _p.acc_thr;

	// commanded motion (pilot sticks or autopilot); NaN = position lock = no motion commanded
	const bool sp_finite = std::isfinite(in.sp_n) && std::isfinite(in.sp_e);
	_s.intent_still = !sp_finite || std::sqrt(in.sp_n * in.sp_n + in.sp_e * in.sp_e) < _p.sp_thr;

	const bool vel_finite = in.vel_valid && std::isfinite(in.vel_n) && std::isfinite(in.vel_e);

	if (in.vel_aided && vel_finite) {
		_s.velocity_still = std::sqrt(in.vel_n * in.vel_n + in.vel_e * in.vel_e) < _p.vel_thr;
		_s.verified = true;

	} else {
		_s.velocity_still = true; // speed not observable: assume the conservative case
		_s.verified = false;
	}

	const bool raw = _s.intent_still && _s.unaccelerated && _s.velocity_still;

	if (raw) {
		if (!_raw_active) {
			_raw_active = true;
			_t_raw = in.now_us;
		}

		_s.hover = elapsed(in.now_us, _t_raw, _p.t_confirm_s);

	} else {
		_raw_active = false;
		_s.hover = false;
	}

	// line-of-sight rotation about the beacon
	const bool rel_finite = std::isfinite(in.rel_n) && std::isfinite(in.rel_e);
	const float rho = rel_finite ? std::sqrt(in.rel_n * in.rel_n + in.rel_e * in.rel_e) : NAN;

	if (!in.has_beacon || !rel_finite) {
		// beacon position unknown: hover is the dominant static-LOS case
		_s.los_rate = NAN;
		_s.los_static = _s.hover;

	} else if (rho <= _p.rho_min) {
		// directly above the beacon (blind cone): no horizontal direction is observed
		_s.los_rate = NAN;
		_s.los_static = true;

	} else if (in.vel_aided && vel_finite) {
		// tangential speed |v x u| over the horizontal distance
		const float v_t = std::fabs(in.vel_n * in.rel_e - in.vel_e * in.rel_n) / rho;
		_s.los_rate = v_t / rho;
		_s.los_static = _s.los_rate * _p.los_window_s < _p.los_angle_min_rad;

	} else {
		// rotation not observable without a velocity aid: conservative
		_s.los_rate = NAN;
		_s.los_static = true;
	}

	return _s;
}

} // namespace gdh
