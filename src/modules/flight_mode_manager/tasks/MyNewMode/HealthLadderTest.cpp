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

TEST_F(HealthLadderTest, SingleRangeIsRankDeficientOnlyAtHover)
{
	// one beacon range plus velocity aiding, no absolute position
	in.pos_aiding = false;
	in.range_only_aiding = true;
	in.hover = false; // moving: the line of sight sweeps, position observable
	run(2.f);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);

	in.hover = true; // hover: tangential direction unobservable
	run(1.f);
	EXPECT_EQ(ladder.level(), Health::DEGRADED);
	EXPECT_TRUE(ladder.reasons() & HOVER_RANK1);
}

TEST_F(HealthLadderTest, NanEphIsIgnored)
{
	in.eph = NAN;
	EXPECT_EQ(run(2.f), 0);
	EXPECT_EQ(ladder.level(), Health::HEALTHY);
}
