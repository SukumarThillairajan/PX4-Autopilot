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
 * @file FlightTaskMyNewMode.hpp
 *
 * GPS-denied hold: Position mode plus a supervisory navigation-health ladder
 * (HEALTHY -> DEGRADED -> FAILING -> LOST) that watches EKF2's aiding status
 * and innovation test ratios, and a hover / line-of-sight detector
 * (gdh::HoverDetector) that works for pilot and autopilot setpoints alike.
 *
 *   HEALTHY   Position mode unchanged.
 *   DEGRADED  horizontal stick speed scaled by GDH_VEL_DEG.
 *   FAILING   horizontal speed capped at GDH_VEL_FAIL, position lock kept.
 *   LOST      horizontal position no longer trusted: sticks command tilt
 *             (Altitude-mode behaviour), altitude hold kept.
 *
 * Every transition is printed and published on gps_denied_hold_status.
 * The commander's own failsafes are unchanged and remain the final authority.
 */

#pragma once

#include "FlightTaskManualPosition.hpp"
#include "HealthLadder.hpp"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/gps_denied_hold_status.h>

class FlightTaskMyNewMode : public FlightTaskManualPosition
{
public:
	FlightTaskMyNewMode() = default;
	virtual ~FlightTaskMyNewMode() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

protected:
	void _scaleSticks() override;
	void _updateSetpoints() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualPosition,
					(ParamFloat<px4::params::GDH_TR_DEG>) _param_gdh_tr_deg,
					(ParamFloat<px4::params::GDH_T_REJ>) _param_gdh_t_rej,
					(ParamFloat<px4::params::GDH_EPH_DEG>) _param_gdh_eph_deg,
					(ParamFloat<px4::params::GDH_EPH_FAIL>) _param_gdh_eph_fail,
					(ParamFloat<px4::params::GDH_EPH_LOST>) _param_gdh_eph_lost,
					(ParamFloat<px4::params::GDH_T_LOST>) _param_gdh_t_lost,
					(ParamFloat<px4::params::GDH_T_ESC>) _param_gdh_t_esc,
					(ParamFloat<px4::params::GDH_T_REC>) _param_gdh_t_rec,
					(ParamFloat<px4::params::GDH_VEL_DEG>) _param_gdh_vel_deg,
					(ParamFloat<px4::params::GDH_VEL_FAIL>) _param_gdh_vel_fail,
					(ParamFloat<px4::params::GDH_HOV_ACC>) _param_gdh_hov_acc,
					(ParamFloat<px4::params::GDH_HOV_VEL>) _param_gdh_hov_vel,
					(ParamFloat<px4::params::GDH_HOV_T>) _param_gdh_hov_t,
					(ParamFloat<px4::params::GDH_LOS_T>) _param_gdh_los_t,
					(ParamFloat<px4::params::GDH_LOS_ANG>) _param_gdh_los_ang,
					(ParamInt<px4::params::GDH_BCN_EN>) _param_gdh_bcn_en,
					(ParamFloat<px4::params::GDH_BCN_N>) _param_gdh_bcn_n,
					(ParamFloat<px4::params::GDH_BCN_E>) _param_gdh_bcn_e,
					(ParamInt<px4::params::EKF2_NOAID_TOUT>) _param_ekf2_noaid_tout
				       )

private:
	void _loadParams();
	void _updateEstimatorStatus();
	void _updateHoverDetector();
	void _updateHealth();
	void _publishStatus(bool transition);

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _estimator_test_ratios_sub{ORB_ID(estimator_innovation_test_ratios)};
	uORB::Publication<gps_denied_hold_status_s> _status_pub{ORB_ID(gps_denied_hold_status)};

	gdh::HealthLadder _ladder{};
	gdh::LadderInputs _inputs{};

	gdh::HoverDetector _hover_detector{};

	estimator_status_flags_s _flags{};
	estimator_innovations_s _test_ratios{}; ///< estimator_innovation_test_ratios uses this layout
	uint8_t _estimator_instance{0};
	uint8_t _xy_reset_counter_seen{0};
	bool _xy_reset_counter_valid{false};

	matrix::Vector2f _last_vel_sp_xy{NAN, NAN}; ///< horizontal velocity setpoint of the previous cycle

	float _speed_limit{NAN};
	hrt_abstime _last_status_publish{0};
};
