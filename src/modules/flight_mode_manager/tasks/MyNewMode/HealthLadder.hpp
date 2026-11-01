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

/**
 * @file HealthLadder.hpp
 *
 * Supervisory navigation-health ladder for GPS-denied position hold:
 * HEALTHY -> DEGRADED -> FAILING -> LOST.
 *
 * Pure logic, no uORB: the flight task fills Inputs from EKF2 topics and the
 * hover detector, and acts on the returned level. It does not fuse anything;
 * it decides how much the horizontal position estimate can be trusted.
 *
 * Escalation (to a worse level) must persist for t_escalate_s, de-escalation
 * (to a better level) for t_recover_s, so that a single noisy sample cannot
 * make the mode chatter. LOST is entered immediately when xy becomes invalid.
 */

#pragma once

#include <stdint.h>

namespace gdh
{

enum class Health : uint8_t {
	HEALTHY = 0,
	DEGRADED = 1,
	FAILING = 2,
	LOST = 3,
};

const char *healthName(Health h);

// Reason bits, identical to GpsDeniedHoldStatus.msg REASON_*
enum Reason : uint16_t {
	NO_POS_AIDING = 1,
	NO_VEL_AIDING = 2,
	DEAD_RECKONING = 4,
	TEST_RATIO_HIGH = 8,
	REJECTING = 16,
	EPH_HIGH = 32,
	XY_INVALID = 64,
	POS_RESET = 128,
	DR_TIMEOUT = 256,
	HOVER_RANK1 = 512,
};

struct LadderParams {
	float tr_degraded{0.5f};   ///< position test ratio above which aiding is marginal (GDH_TR_DEG)
	float t_reject_s{2.f};     ///< sustained rejection (ratio > 1) before FAILING (GDH_T_REJ)
	float eph_degraded{5.f};   ///< eph above which DEGRADED (GDH_EPH_DEG), m
	float eph_failing{15.f};   ///< eph above which FAILING (GDH_EPH_FAIL), m
	float eph_lost{50.f};      ///< eph above which LOST (GDH_EPH_LOST), m
	float t_lost_s{3.f};       ///< dead reckoning longer than this -> LOST (GDH_T_LOST), s
	float t_escalate_s{0.5f};  ///< persistence before moving to a worse level (GDH_T_ESC), s
	float t_recover_s{5.f};    ///< persistence before moving to a better level (GDH_T_REC), s
	float t_reset_hold_s{10.f}; ///< a position reset keeps at least DEGRADED for this long, s
};

struct LadderInputs {
	uint64_t now_us{0};
	bool xy_valid{false};
	float eph{0.f};
	bool pos_aiding{false};        ///< GNSS pos / EV pos / aux global pos fused
	bool vel_aiding{false};        ///< GNSS vel / EV vel / optical flow fused
	bool range_only_aiding{false}; ///< rank-1 aiding (single beacon); no EKF2 source yet
	bool dead_reckoning{false};    ///< EKF cs_inertial_dead_reckoning
	float pos_test_ratio{0.f};     ///< largest fused horizontal position test ratio
	bool pos_reset{false};         ///< xy reset counter changed this cycle
	bool hover{false};             ///< hover regime from the hover detector
};

class HealthLadder
{
public:
	void setParams(const LadderParams &p) { _p = p; }
	const LadderParams &params() const { return _p; }

	/** Start (or restart) at HEALTHY-candidate evaluation; the first update sets the level directly. */
	void reset(uint64_t now_us);

	/** Evaluate one cycle. Returns true if the level changed. */
	bool update(const LadderInputs &in);

	Health level() const { return _level; }
	Health previous() const { return _prev; }
	Health candidate() const { return _candidate; }
	uint16_t reasons() const { return _reasons; }
	uint16_t transitions() const { return _transitions; }
	uint64_t timeInLevelUs(uint64_t now_us) const { return now_us - _t_level_start; }
	float deadReckoningSeconds(uint64_t now_us) const;

	/** Instantaneous classification, exposed for unit tests. */
	Health classify(const LadderInputs &in, uint16_t &reasons);

private:
	LadderParams _p{};
	Health _level{Health::HEALTHY};
	Health _prev{Health::HEALTHY};
	Health _candidate{Health::HEALTHY};
	uint16_t _reasons{0};
	uint16_t _transitions{0};
	bool _initialized{false};

	uint64_t _t_level_start{0};
	uint64_t _t_candidate_start{0};
	uint64_t _t_dr_start{0};      ///< 0 while aided
	uint64_t _t_reject_start{0};  ///< 0 while innovations pass
	uint64_t _t_last_reset{0};    ///< 0 if no reset seen
};

} // namespace gdh
