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

void HealthLadder::reset(uint64_t now_us)
{
	_level = Health::HEALTHY;
	_prev = Health::HEALTHY;
	_candidate = Health::HEALTHY;
	_reasons = 0;
	_transitions = 0;
	_initialized = false;
	_t_level_start = now_us;
	_t_candidate_start = now_us;
	_t_dr_start = 0;
	_t_reject_start = 0;
	_t_last_reset = 0;
}

float HealthLadder::deadReckoningSeconds(uint64_t now_us) const
{
	return (_t_dr_start == 0 || now_us < _t_dr_start) ? 0.f : (now_us - _t_dr_start) / US_PER_S;
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

	// A single range observes only its line of sight: acceptable while moving, rank-1 at hover.
	const bool range_counts = in.range_only_aiding && !in.hover;
	const bool any_aiding = in.pos_aiding || in.vel_aiding || in.range_only_aiding;

	// timers
	if (in.dead_reckoning || !any_aiding) {
		if (_t_dr_start == 0) {
			_t_dr_start = in.now_us;
		}

	} else {
		_t_dr_start = 0;
	}

	const bool rejecting = in.pos_aiding && (in.pos_test_ratio > 1.f);

	if (rejecting) {
		if (_t_reject_start == 0) {
			_t_reject_start = in.now_us;
		}

	} else {
		_t_reject_start = 0;
	}

	if (in.pos_reset) {
		_t_last_reset = in.now_us;
	}

	const bool eph_finite = std::isfinite(in.eph);

	// LOST
	if (!in.xy_valid) {
		raise(Health::LOST, XY_INVALID);
	}

	if (eph_finite && in.eph > _p.eph_lost) {
		raise(Health::LOST, EPH_HIGH);
	}

	if (_t_dr_start != 0 && deadReckoningSeconds(in.now_us) > _p.t_lost_s) {
		raise(Health::LOST, DR_TIMEOUT);
	}

	// FAILING
	if (_t_dr_start != 0) {
		raise(Health::FAILING, in.dead_reckoning ? DEAD_RECKONING : (NO_POS_AIDING | NO_VEL_AIDING));
	}

	if (eph_finite && in.eph > _p.eph_failing) {
		raise(Health::FAILING, EPH_HIGH);
	}

	if (rejecting && (in.now_us - _t_reject_start) > _p.t_reject_s * US_PER_S) {
		raise(Health::FAILING, REJECTING);
	}

	// DEGRADED
	if (!in.pos_aiding && !range_counts) {
		raise(Health::DEGRADED, in.range_only_aiding ? HOVER_RANK1 : NO_POS_AIDING);
	}

	if (!in.vel_aiding) {
		reasons |= NO_VEL_AIDING; // informative only: position aiding alone still bounds the drift
	}

	if (in.pos_aiding && in.pos_test_ratio > _p.tr_degraded) {
		raise(Health::DEGRADED, TEST_RATIO_HIGH);
	}

	if (eph_finite && in.eph > _p.eph_degraded) {
		raise(Health::DEGRADED, EPH_HIGH);
	}

	if (_t_last_reset != 0 && (in.now_us - _t_last_reset) < _p.t_reset_hold_s * US_PER_S) {
		raise(Health::DEGRADED, POS_RESET);
	}

	return h;
}

bool HealthLadder::update(const LadderInputs &in)
{
	uint16_t reasons = 0;
	const Health c = classify(in, reasons);
	_reasons = reasons;

	if (!_initialized) {
		_initialized = true;
		_level = c;
		_prev = c;
		_candidate = c;
		_t_level_start = in.now_us;
		_t_candidate_start = in.now_us;
		return false;
	}

	if (c != _candidate) {
		_candidate = c;
		_t_candidate_start = in.now_us;
	}

	if (_candidate == _level) {
		return false;
	}

	const bool worse = _candidate > _level;
	const bool immediate = worse && (reasons & XY_INVALID);
	const float need_s = worse ? _p.t_escalate_s : _p.t_recover_s;

	if (immediate || (in.now_us - _t_candidate_start) >= need_s * US_PER_S) {
		_prev = _level;

		// escalate straight to the candidate, recover one level at a time
		_level = worse ? _candidate : static_cast<Health>(static_cast<uint8_t>(_level) - 1);
		_t_level_start = in.now_us;
		_t_candidate_start = in.now_us; // next recovery step needs its own persistence
		++_transitions;
		return true;
	}

	return false;
}

} // namespace gdh
