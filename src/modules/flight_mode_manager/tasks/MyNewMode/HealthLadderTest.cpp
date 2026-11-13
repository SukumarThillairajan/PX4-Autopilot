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

#include <gtest/gtest.h>

#include <cmath>

#include "HealthLadder.hpp"

using namespace gdh;

static constexpr uint64_t T0 = 1'000'000; // 1 s after boot
static constexpr uint64_t DT = 20'000;    // 50 Hz task rate

// ---------------------------------------------------------------------------
// HealthLadder
// ---------------------------------------------------------------------------

class HealthLadderTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		ladder.reset(T0);
		in = healthy();
		now = T0;
		step(); // first update latches the level directly
	}

	static LadderInputs healthy()
	{
		LadderInputs i{};
		i.estimator_age_s = 0.1f;
		i.xy_valid = true;
		i.eph = 1.f;
		i.pos_aiding = true;
		i.vel_aiding = true;
		i.pos_test_ratio = 0.1f;
		return i;
	}

	bool step()
	{
		now += DT;
		in.now_us = now;
		return ladder.update(in);
	}

	// run for `seconds`, return the number of transitions seen
	int run(float seconds)
	{
		int n = 0;
		const uint64_t end = now + static_cast<uint64_t>(seconds * 1e6f);

		while (now < end) {
			n += step() ? 1 : 0;
		}

		return n;
	}

	HealthLadder ladder;
	LadderInputs in;
	uint64_t now{T0};
};

TEST_F(HealthLadderTest, StaysHealthyWithGoodAiding)
{
	EXPECT_EQ(run(10.f), 0);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
	EXPECT_EQ(ladder.reasons(), 0);
}

TEST_F(HealthLadderTest, VelocityOnlyAidingIsDegraded)
{
	in.pos_aiding = false; // e.g. optical flow only: position drifts as b_v * t
	run(0.4f);
	EXPECT_EQ(ladder.level(), Health::HEALTHY) << "escalation must persist GDH_T_ESC";
	run(0.2f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	EXPECT_TRUE(ladder.reasons() & NO_POS_AIDING);
}

TEST_F(HealthLadderTest, GpsLossWalksDownTheLadder)
{
	in.pos_aiding = false;
	in.vel_aiding = false;
	in.dead_reckoning = true;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::FAILING);
	EXPECT_TRUE(ladder.reasons() & DEAD_RECKONING);
	EXPECT_GT(ladder.deadReckoningSeconds(now), 0.9f);

	run(3.f); // dead reckoning now longer than GDH_T_LOST = 3 s (+ escalation delay)
	EXPECT_EQ(ladder.level(), Health::LOST);
	EXPECT_TRUE(ladder.reasons() & DR_TIMEOUT);
	EXPECT_EQ(ladder.transitions(), 2);
}

TEST_F(HealthLadderTest, InvalidXyIsImmediatelyLost)
{
	in.xy_valid = false;
	EXPECT_TRUE(step());
	EXPECT_EQ(ladder.level(), Health::LOST);
	EXPECT_EQ(ladder.previous(), Health::HEALTHY);
}

TEST_F(HealthLadderTest, RecoveryIsOneLevelAtATime)
{
	in.xy_valid = false;
	step();
	ASSERT_EQ(ladder.level(), Health::LOST);

	in = healthy();
	run(4.9f);
	EXPECT_EQ(ladder.level(), Health::LOST) << "recovery must persist GDH_T_REC";
	run(0.2f);
	EXPECT_EQ(ladder.level(), Health::FAILING);
	run(5.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	run(5.f);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
}

TEST_F(HealthLadderTest, SingleNoisySampleDoesNotChatter)
{
	in.pos_test_ratio = 0.9f;
	step();
	in.pos_test_ratio = 0.1f;
	EXPECT_EQ(run(10.f), 0);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
}

TEST_F(HealthLadderTest, FlickeringWorseCandidatesStillEscalate)
{
	// the candidate alternates between DEGRADED and FAILING every cycle; both are worse than
	// HEALTHY, so the escalation timer must keep running and the ladder must leave HEALTHY
	for (int i = 0; i < 50; ++i) {
		in.eph = (i % 2 == 0) ? 6.f : 20.f;
		step();
	}

	EXPECT_NE(ladder.level(), Health::HEALTHY);
}

TEST_F(HealthLadderTest, HighTestRatioDegradesSustainedRejectionFails)
{
	// spoof-like: innovations grow, then are rejected by the EKF gate
	in.pos_test_ratio = 0.7f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	EXPECT_TRUE(ladder.reasons() & TEST_RATIO_HIGH);

	in.pos_test_ratio = 3.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED) << "rejection must last GDH_T_REJ";
	run(2.f);
	EXPECT_EQ(ladder.level(), Health::FAILING);
	EXPECT_TRUE(ladder.reasons() & REJECTING);
}

TEST_F(HealthLadderTest, PositionResetHoldsDegraded)
{
	in.pos_reset = true; // EKF jumped its position: possible capture by a spoofer
	step();
	in.pos_reset = false;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	EXPECT_TRUE(ladder.reasons() & POS_RESET);
	run(14.1f); // 10 s hold + 5 s recovery
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
}

TEST_F(HealthLadderTest, EphThresholds)
{
	in.eph = 6.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	in.eph = 20.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::FAILING);
	in.eph = 60.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::LOST);
	EXPECT_TRUE(ladder.reasons() & EPH_HIGH);
}

TEST_F(HealthLadderTest, NanEphIsIgnored)
{
	in.eph = NAN;
	EXPECT_EQ(run(2.f), 0);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
}

TEST_F(HealthLadderTest, StaleEstimatorStatusIsFailing)
{
	// estimator flags stop arriving: aiding can no longer be assumed
	in.estimator_age_s = 5.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::FAILING);
	EXPECT_TRUE(ladder.reasons() & ESTIMATOR_STALE);

	in.estimator_age_s = NAN; // never received
	run(4.f);
	EXPECT_EQ(ladder.level(), Health::LOST) << "stale flags count as no aiding: dead-reckoning timeout";
}

TEST_F(HealthLadderTest, MisorderedEphParametersAreSanitised)
{
	LadderParams p{};
	p.eph_degraded = 20.f;
	p.eph_failing = 10.f; // below degraded
	p.eph_lost = 5.f;     // below failing
	ladder.setParams(p);
	EXPECT_LE(ladder.params().eph_degraded, ladder.params().eph_failing);
	EXPECT_LE(ladder.params().eph_failing, ladder.params().eph_lost);

	// sanitised to 20 / 20 / 20 m: below them nothing happens, above them the worst level wins
	in.eph = 15.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
	in.eph = 25.f;
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::LOST);
}

TEST_F(HealthLadderTest, SingleRangeIsRankDeficientWhileLosIsStatic)
{
	// one beacon range plus velocity aiding, no absolute position
	in.pos_aiding = false;
	in.range_only_aiding = true;
	in.los_static = false; // the line of sight sweeps: second direction observed over time
	run(2.f);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);

	in.los_static = true; // hover, or radial motion: tangential direction unobservable
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	EXPECT_TRUE(ladder.reasons() & RANGE_RANK1);
}

// ---------------------------------------------------------------------------
// HoverDetector
// ---------------------------------------------------------------------------

class HoverDetectorTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		det.reset();
		in = HoverInputs{};
		in.dt_s = DT * 1e-6f;
		in.vel_valid = true;
		in.vel_aided = true;
		now = T0;
	}

	const HoverState &run(float seconds)
	{
		const uint64_t end = now + static_cast<uint64_t>(seconds * 1e6f);

		while (now < end) {
			now += DT;
			in.now_us = now;
			det.update(in);
		}

		return det.state();
	}

	HoverDetector det;
	HoverInputs in;
	uint64_t now{T0};
};

TEST_F(HoverDetectorTest, StillVehicleIsHoverAfterConfirmation)
{
	EXPECT_FALSE(run(0.5f).hover);
	const HoverState &s = run(1.f);
	EXPECT_TRUE(s.hover);
	EXPECT_TRUE(s.verified);
}

TEST_F(HoverDetectorTest, AutopilotConstantVelocityIsNotHover)
{
	// no pilot: the autopilot commands 3 m/s and the vehicle flies it, unaccelerated
	in.sp_n = 3.f;
	in.vel_n = 3.f;
	const HoverState &s = run(3.f);
	EXPECT_TRUE(s.unaccelerated) << "the accelerometer alone cannot tell this from hover";
	EXPECT_FALSE(s.hover);
}

TEST_F(HoverDetectorTest, UncommandedSteadyDriftIsNotHoverWhenVelocityIsAided)
{
	// nothing commanded, but an aided velocity shows a steady 1 m/s drift (e.g. wind)
	in.vel_n = 1.f;
	EXPECT_FALSE(run(3.f).hover);
}

TEST_F(HoverDetectorTest, UnaidedVelocityIsAssumedStillButNotVerified)
{
	in.vel_aided = false;
	in.vel_n = 1.f; // dead-reckoned: cannot be trusted either way
	const HoverState &s = run(3.f);
	EXPECT_TRUE(s.hover);
	EXPECT_FALSE(s.verified);
}

TEST_F(HoverDetectorTest, AccelerationBreaksHover)
{
	in.acc_n = 1.f;
	EXPECT_FALSE(run(3.f).hover);
}

TEST_F(HoverDetectorTest, RadialMotionLeavesLosStatic)
{
	// 1 km north of the beacon, flying 3 m/s straight away from it: not hover, LOS static
	in.has_beacon = true;
	in.rel_n = 1000.f;
	in.sp_n = 3.f;
	in.vel_n = 3.f;
	const HoverState &s = run(2.f);
	EXPECT_FALSE(s.hover);
	EXPECT_TRUE(s.los_static);
	EXPECT_NEAR(s.los_rate, 0.f, 1e-6f);
}

TEST_F(HoverDetectorTest, TangentialMotionRotatesLos)
{
	// 1 km north, flying 3 m/s east: 3 mrad/s, i.e. 1.7 deg in 10 s, below the 3 deg minimum
	in.has_beacon = true;
	in.rel_n = 1000.f;
	in.sp_e = 3.f;
	in.vel_e = 3.f;
	EXPECT_TRUE(run(2.f).los_static);

	// 10 m/s east: 0.1 rad = 5.7 deg in 10 s, enough rotation
	in.sp_e = 10.f;
	in.vel_e = 10.f;
	const HoverState &s = run(2.f);
	EXPECT_FALSE(s.los_static);
	EXPECT_NEAR(s.los_rate, 0.01f, 1e-4f);
}

TEST_F(HoverDetectorTest, AboveTheBeaconIsStatic)
{
	in.has_beacon = true;
	in.rel_n = 0.5f; // inside the blind cone
	in.sp_e = 5.f;
	in.vel_e = 5.f;
	EXPECT_TRUE(run(1.f).los_static);
}

TEST_F(HoverDetectorTest, WithoutBeaconLosFollowsHover)
{
	in.has_beacon = false;
	EXPECT_EQ(run(2.f).los_static, det.state().hover);
	in.sp_n = 2.f;
	in.vel_n = 2.f;
	EXPECT_FALSE(run(2.f).los_static);
}
