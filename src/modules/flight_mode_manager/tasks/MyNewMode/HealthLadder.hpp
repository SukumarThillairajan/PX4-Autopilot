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
 * Pure logic for the GPS-denied hold mode, with no uORB or parameter access,
 * so that it can be unit-tested in isolation:
 *
 *  - HealthLadder: supervisory navigation-health ladder
 *    HEALTHY -> DEGRADED -> FAILING -> LOST.
 *  - HoverDetector: hover-regime and line-of-sight (LOS) rotation detector.
 *
 * The flight task fills the inputs from EKF2 topics and acts on the outputs.
 * Neither class fuses measurements: they decide how far the horizontal
 * position estimate can be trusted.
 *
 * Hysteresis of the ladder: a worse level is entered only after the candidate
 * has stayed worse than the current level for t_escalate_s (the jump goes
 * straight to the candidate); a better level is entered only after the
 * candidate has stayed better for t_recover_s, and then only one level at a
 * time. LOST is entered at once when the local position becomes invalid.
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
	RANGE_RANK1 = 512,
	ESTIMATOR_STALE = 1024,
};

struct LadderParams {
	float tr_degraded{0.5f};    ///< position test ratio above which aiding is marginal (GDH_TR_DEG)
	float t_reject_s{2.f};      ///< sustained rejection (ratio > 1) before FAILING (GDH_T_REJ)
	float eph_degraded{5.f};    ///< eph above which DEGRADED (GDH_EPH_DEG), m
	float eph_failing{15.f};    ///< eph above which FAILING (GDH_EPH_FAIL), m
	float eph_lost{50.f};       ///< eph above which LOST (GDH_EPH_LOST), m
	float t_lost_s{3.f};        ///< dead reckoning longer than this -> LOST (GDH_T_LOST), s
	float t_escalate_s{0.5f};   ///< persistence before moving to a worse level (GDH_T_ESC), s
	float t_recover_s{5.f};     ///< persistence before moving to a better level (GDH_T_REC), s
	float t_reset_hold_s{10.f}; ///< a position reset keeps at least DEGRADED for this long, s
	float t_stale_s{3.f};       ///< estimator status older than this is treated as unknown, s

	/** Clamp to valid ranges and enforce eph_degraded <= eph_failing <= eph_lost. */
	void sanitize();
};

struct LadderInputs {
	uint64_t now_us{0};
	float estimator_age_s{0.f};    ///< age of the latest EKF status flags; NaN if none received
	bool xy_valid{false};          ///< local position xy and vxy valid
	float eph{0.f};
	bool pos_aiding{false};        ///< GNSS pos / EV pos / aux global pos fused
	bool vel_aiding{false};        ///< GNSS vel / EV vel / optical flow / wind-relative fused
	bool range_only_aiding{false}; ///< rank-1 aiding (single beacon); no EKF2 source yet
	bool dead_reckoning{false};    ///< EKF cs_inertial_dead_reckoning
	float pos_test_ratio{0.f};     ///< largest fused horizontal position test ratio
	bool pos_reset{false};         ///< xy reset counter changed this cycle
	bool los_static{false};        ///< beacon line of sight not rotating (HoverDetector)
};

class HealthLadder
{
public:
	void setParams(const LadderParams &p);
	const LadderParams &params() const { return _p; }

	/** Restart; the first update after a reset latches the level directly. */
	void reset(uint64_t now_us);

	/** Evaluate one cycle. Returns true if the level changed. */
	bool update(const LadderInputs &in);

	Health level() const { return _level; }
	Health previous() const { return _prev; }
	Health candidate() const { return _candidate; }
	uint16_t reasons() const { return _reasons; }
	uint16_t transitions() const { return _transitions; }
	uint64_t timeInLevelUs(uint64_t now_us) const { return (now_us > _t_level_start) ? now_us - _t_level_start : 0; }
	float deadReckoningSeconds(uint64_t now_us) const;

	/** Instantaneous classification (updates the internal timers), exposed for unit tests. */
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

	bool _worse_active{false};    ///< candidate has been worse than the level since _t_worse
	uint64_t _t_worse{0};
	bool _better_active{false};   ///< candidate has been better than the level since _t_better
	uint64_t _t_better{0};

	bool _dr_active{false};       ///< no horizontal aiding since _t_dr
	uint64_t _t_dr{0};
	bool _reject_active{false};   ///< position innovations rejected since _t_reject
	uint64_t _t_reject{0};
	bool _reset_seen{false};      ///< an EKF position reset happened at _t_reset
	uint64_t _t_reset{0};
};

struct HoverParams {
	float acc_thr{0.3f};          ///< low-passed |a_xy| below this is unaccelerated (GDH_HOV_ACC), m/s^2
	float vel_thr{0.3f};          ///< aided |v_xy| below this is still (GDH_HOV_VEL), m/s
	float sp_thr{0.1f};           ///< |velocity setpoint| below this means no commanded motion, m/s
	float t_confirm_s{1.f};       ///< conditions must hold this long (GDH_HOV_T), s
	float lpf_tau_s{0.5f};        ///< acceleration low-pass time constant, s
	float los_window_s{10.f};     ///< window over which the LOS must sweep los_angle_min_rad (GDH_LOS_T), s
	float los_angle_min_rad{0.0524f}; ///< 3 deg (GDH_LOS_ANG)
	float rho_min{1.f};           ///< below this horizontal distance to the beacon the LOS is undefined, m
};

struct HoverInputs {
	uint64_t now_us{0};
	float dt_s{0.f};
	float acc_n{0.f}, acc_e{0.f};   ///< EKF (IMU-derived) NED horizontal acceleration
	float vel_n{0.f}, vel_e{0.f};   ///< EKF NED horizontal velocity
	bool vel_valid{false};
	bool vel_aided{false};          ///< velocity observed by an aid, not only integrated
	float sp_n{0.f}, sp_e{0.f};     ///< commanded horizontal velocity (pilot or autopilot); NaN = position lock
	bool has_beacon{false};
	float rel_n{0.f}, rel_e{0.f};   ///< drone minus beacon, horizontal, m
};

struct HoverState {
	float acc_filt{0.f};            ///< |low-passed a_xy|
	bool unaccelerated{false};
	bool intent_still{false};       ///< no horizontal motion commanded
	bool velocity_still{false};     ///< aided speed small, or speed not observable (conservative)
	bool verified{false};           ///< velocity_still came from an aided velocity
	bool hover{false};              ///< hover regime, confirmed for t_confirm_s
	float los_rate{0.f};            ///< LOS angular rate about the beacon, rad/s; NaN if unknown
	bool los_static{false};         ///< LOS sweeps less than los_angle_min_rad over los_window_s
};

/**
 * Hover regime = no commanded horizontal motion AND unaccelerated AND (if the velocity is
 * observed by an aid) slow, held for t_confirm_s. The setpoint, not the sticks, carries the
 * intent, so the test is the same for a pilot and for an autopilot. An accelerometer alone
 * cannot separate hover from constant-velocity flight; when no velocity aid is fused, the
 * speed is unverifiable and the detector assumes the conservative answer (still).
 *
 * For a single range what matters is not hover itself but whether the line of sight to the
 * beacon rotates: hover, and constant-velocity motion straight towards or away from the
 * beacon, both leave it static. los_static uses the beacon position when it is configured
 * and falls back to the hover flag when it is not.
 */
class HoverDetector
{
public:
	void setParams(const HoverParams &p) { _p = p; }
	void reset();
	const HoverState &update(const HoverInputs &in);
	const HoverState &state() const { return _s; }

private:
	HoverParams _p{};
	HoverState _s{};
	float _lpf_n{0.f}, _lpf_e{0.f};
	bool _raw_active{false};
	uint64_t _t_raw{0};
};

} // namespace gdh
