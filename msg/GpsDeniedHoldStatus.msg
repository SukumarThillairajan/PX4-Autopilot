# Supervisory navigation-health ladder of the GPS-denied hold mode (FlightTaskMyNewMode).
# Published on every transition and at 2 Hz while the mode is active.

uint64 timestamp                 # time since system start (microseconds)

uint8 HEALTH_HEALTHY = 0         # absolute horizontal position aiding fused and consistent
uint8 HEALTH_DEGRADED = 1        # position aiding marginal or rank-deficient, or velocity-only aiding
uint8 HEALTH_FAILING = 2         # no horizontal aiding (inertial dead reckoning) or aiding rejected
uint8 HEALTH_LOST = 3            # horizontal position no longer trusted: altitude-hold fallback

uint8 health                     # current level (HEALTH_*)
uint8 health_prev                # level before the last transition
uint8 candidate                  # instantaneous level before hysteresis (HEALTH_*)
uint16 transitions               # number of transitions since activation
uint64 time_in_level_us          # time spent in the current level

uint16 REASON_NO_POS_AIDING = 1      # no absolute horizontal position source fused
uint16 REASON_NO_VEL_AIDING = 2      # no horizontal velocity source fused
uint16 REASON_DEAD_RECKONING = 4     # EKF reports inertial dead reckoning
uint16 REASON_TEST_RATIO_HIGH = 8    # position innovation test ratio above GDH_TR_DEG
uint16 REASON_REJECTING = 16         # position innovations rejected (test ratio > 1) for longer than GDH_T_REJ
uint16 REASON_EPH_HIGH = 32          # eph above GDH_EPH_DEG / GDH_EPH_FAIL / GDH_EPH_LOST
uint16 REASON_XY_INVALID = 64        # local position xy not valid
uint16 REASON_POS_RESET = 128        # EKF horizontal position reset while in the mode (possible spoof capture)
uint16 REASON_DR_TIMEOUT = 256       # dead reckoning longer than GDH_T_LOST
uint16 REASON_HOVER_RANK1 = 512      # hover regime with range-only (rank-1) aiding: tangential drift unobservable

uint16 reasons                   # REASON_* bitmask that produced the candidate level

bool hover                       # hover regime: unaccelerated (|a_h| < GDH_HOV_ACC) with centred sticks for GDH_HOV_T
float32 accel_xy_filt            # low-passed horizontal acceleration magnitude used by the hover detector (m/s^2)
bool pos_aiding                  # absolute horizontal position source fused (GNSS / external vision / aux global position)
bool vel_aiding                  # horizontal velocity source fused (GNSS velocity / external vision velocity / optical flow)
bool range_only_aiding           # rank-1 aiding (single range beacon); false until a beacon source exists in EKF2
float32 pos_test_ratio           # largest horizontal position innovation test ratio of the fused sources (<1 passes the gate)
float32 eph                      # EKF horizontal position standard deviation (m)
float32 dead_reckoning_s         # time since horizontal aiding was lost (s), 0 while aided
float32 speed_limit              # horizontal speed limit applied by the mode (m/s), NaN when not position-controlled
