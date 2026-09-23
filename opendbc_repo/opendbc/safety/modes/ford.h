#pragma once

#include "opendbc/safety/declarations.h"

// StarPilot's extended Ford curvature enforcement below is substantially adapted from
// BluePilot bp-7.0 panda work, principally Alan Polk's 8f8d6d15f0a590f42b78de964ffb0d0af7f5d63d
// See /CREDITS.md and /THIRD_PARTY_NOTICES.md. This comment does not attribute the surrounding
// upstream openpilot code.


// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_SteeringPinion_Data   0x07EU   // RX from PSCM, measured steering pinion angle
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  } else {
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  } else {
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  } else {
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  } else if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  } else if (msg->addr == FORD_SteeringPinion_Data) {
    valid = ((msg->data[5] >> 2) & 0x3U) == 0x3U;  // StePinCompAnEst_D_Qf, 3 = OK
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

static bool ford_lka_steering = false;
static bool ford_extended_lateral = false;
static bool ford_longitudinal = false;
static bool ford_cancel_resume_button = false;

// Lateral continuation, LKA_STEERING platforms only. The PCM leaves CcStat_D_Actl Active
// for Standby at about 4.94 m/s on the way to a stop, pcm_cruise_check drops
// controls_allowed, and every command stops - lateral included, although lateral is a
// PSCM channel the PCM has no part in. When enabled, a latch taken on that exact
// transition keeps Lane_Assist_Data1 permitted below it, and nothing else: ACCDATA, the
// cruise buttons and LateralMotionControl all keep gating on controls_allowed alone.
//
// The latch is recomputed here rather than trusted from openpilot, and CarState runs
// the identical entry/exit conditions off the identical signals (opendbc/car/ford/carstate.py
// and the TRANSIT_LKA_CONT_* constants in ford/values.py). The two are deliberately
// asymmetric, not required to mirror each other exactly: this side additionally clears the
// latch on safety_rx_checks_invalid (see ford_rx_hook below), a guard the Python side does
// not repeat. That is safe because a stale EngBrakeData stream already drives cp.can_valid
// false on the openpilot side, which disengages via canError within about 1s regardless of
// what carstate.py's own latch does - panda is simply the stricter of the two, not a second
// copy that has to match bit for bit.
static bool ford_lka_continuation_enabled = false;
static bool ford_lka_continuation = false;
#define FORD_LKA_CONT_ENTER_SPEED     7.0f
#define FORD_LKA_CONT_EXIT_SPEED_HIGH 9.0f
#define FORD_LKA_CONT_EXIT_SPEED_LOW  0.5f
#define FORD_CRUISE_STANDBY           3U

// Curvature rate limits
#define FORD_LIMITS(limit_lateral_acceleration) {                                               \
  .max_angle = 1000,          /* 0.02 curvature */                                              \
  .angle_deg_to_can = 50000,  /* 1 / (2e-5) rad to can */                                       \
  .max_angle_error = 100,     /* 0.002 * FORD_STEERING_LIMITS.angle_deg_to_can */               \
  .angle_rate_up_lookup = {                                                                     \
    {5., 25., 25.},                                                                             \
    {0.00045, 0.0001, 0.0001}                                                                   \
  },                                                                                            \
  .angle_rate_down_lookup = {                                                                   \
    {5., 25., 25.},                                                                             \
    {0.00045, 0.00015, 0.00015}                                                                 \
  },                                                                                            \
                                                                                                \
  /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */ \
  .angle_error_min_speed = 10.0,    /* m/s */                                                   \
                                                                                                \
  .angle_is_curvature = (limit_lateral_acceleration),                                           \
  .enforce_angle_error = true,                                                                  \
  .inactive_angle_is_zero = true,                                                               \
}

static const AngleSteeringLimits FORD_STEERING_LIMITS = FORD_LIMITS(false);

#define FORD_EXTENDED_LIMITS(limit_lateral_acceleration) {                                      \
  .max_angle = 1000,                                                                            \
  .angle_deg_to_can = 50000,                                                                    \
  .max_angle_error = 100,                                                                       \
  .angle_rate_up_lookup = {                                                                     \
    {5., 16., 25.},                                                                             \
    {0.0025, 0.0014, 0.00018}                                                                   \
  },                                                                                            \
  .angle_rate_down_lookup = {                                                                   \
    {5., 16., 25.},                                                                             \
    {0.0025, 0.0014, 0.00018}                                                                   \
  },                                                                                            \
  .angle_error_min_speed = 10.0,                                                                \
  .frequency = 20U,                                                                             \
  .angle_is_curvature = (limit_lateral_acceleration),                                           \
  .enforce_angle_error = true,                                                                  \
  .inactive_angle_is_zero = true,                                                               \
}

static const AngleSteeringLimits FORD_EXTENDED_STEERING_LIMITS = FORD_EXTENDED_LIMITS(false);

static void ford_rx_hook(const CANPacket_t *msg) {
  // The continuation latch is written only from EngBrakeData (see below) and is
  // otherwise never cleared, so a stale EngBrakeData stream (one ECU dropping out, not
  // necessarily a dead bus) would leave it set indefinitely. That would bypass panda's
  // only protection against a stale rx stream, which normally acts through
  // controls_allowed = false (safety_tick, on the same safety_rx_checks_invalid signal).
  // Clear the latch here so it can never outlive a valid rx stream.
  if (safety_rx_checks_invalid) {
    ford_lka_continuation = false;
  }

  if (msg->bus == FORD_MAIN_BUS) {
    // Update in motion state from standstill signal
    if (msg->addr == FORD_DesiredTorqBrk) {
      // Signal: VehStop_D_Stat
      vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
    }

    // Update vehicle speed
    if (msg->addr == FORD_BrakeSysFeatures) {
      // Signal: Veh_V_ActlBrk
      UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
    }

    // Check vehicle speed against a second source
    if (msg->addr == FORD_EngVehicleSpThrottle2) {
      // Disable controls if speeds from ABS and PCM ECUs are too far apart.
      // Signal: Veh_V_ActlEng
      float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
      speed_mismatch_check(filtered_pcm_speed);
    }

    // Update vehicle yaw rate
    // LKA_STEERING platforms track the pinion angle instead; angle_meas must not be fed two unit scales
    if ((msg->addr == FORD_Yaw_Data_FD1) && !ford_lka_steering) {
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&angle_meas, ROUND(current_curvature * FORD_STEERING_LIMITS.angle_deg_to_can));
    }

    // Measured steering pinion angle, used by LKA_STEERING platforms to bound
    // Lane_Assist_Data1 angle requests (see ford_tx_hook)
    if ((msg->addr == FORD_SteeringPinion_Data) && ford_lka_steering) {
      // Signal: StePinComp_An_Est : 22|15@0+ (0.1,-1600) degrees -> tenths of a degree,
      // matching FORD_LKA_DEG_TO_CAN in ford_tx_hook
      const int pinion_angle = (int)(((msg->data[2] & 0x7FU) << 8) | msg->data[3]) - 16000;
      update_sample(&angle_meas, pinion_angle);
    }

    // Update gas pedal
    if (msg->addr == FORD_EngVehicleSpThrottle) {
      // Pedal position: (0.1 * val) in percent
      // Signal: ApedPos_Pc_ActlArb
      gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
    }

    // Update brake pedal and cruise state
    if (msg->addr == FORD_EngBrakeData) {
      // Signal: BpedDrvAppl_D_Actl
      brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

      // Signal: CcStat_D_Actl
      unsigned int cruise_state = msg->data[1] & 0x07U;
      bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);

      // Lateral continuation latch. Evaluated before pcm_cruise_check, which consumes and
      // then overwrites cruise_engaged_prev: entry needs the state from the frame before,
      // so the latch is only taken on the Active -> Standby transition and never from a
      // standing start in Standby. Speed is the unfiltered Veh_V_ActlBrk sample.
      const float ford_speed = ((float)vehicle_speed.values[0]) / VEHICLE_SPEED_FACTOR;
      const bool ford_standby = cruise_state == FORD_CRUISE_STANDBY;
      if (!ford_lka_continuation) {
        ford_lka_continuation = ford_lka_continuation_enabled && cruise_engaged_prev && ford_standby &&
                                (ford_speed < FORD_LKA_CONT_ENTER_SPEED) && !brake_pressed;
      } else {
        ford_lka_continuation = ford_standby && !brake_pressed &&
                                (ford_speed > FORD_LKA_CONT_EXIT_SPEED_LOW) &&
                                (ford_speed < FORD_LKA_CONT_EXIT_SPEED_HIGH);
      }

      pcm_cruise_check(cruise_engaged);

      acc_main_on = (cruise_state == 3U) || cruise_engaged;
    }

    if (msg->addr == FORD_Steering_Data_FD1) {
      ford_cancel_resume_button = ((msg->data[2] >> 5) & 1U) != 0U;
    }
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = ((msg->data[6] >> 6) & 1U) || ((msg->data[6] >> 7) & 1U);

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny; // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    bool stock_resume_from_driver = !ford_longitudinal && acc_main_on && ford_cancel_resume_button;
    violation |= ((msg->data[3] >> 1) & 1U) && !(controls_allowed || stock_resume_from_driver);  // Signal: CcAsllButtnResPress (resume)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Signal: LkaActvStats_D2_Req : 7|3@0+ (1,0), i.e. the top 3 bits of byte 0
    unsigned int action = msg->data[0] >> 5;

    if (!ford_lka_steering) {
      // Curvature platforms steer through LCA/TJA; this frame must never carry an action.
      if (action != 0U) {
        tx = false;
      }
      ford_extended_lateral = (msg->data[4] & 0x2U) != 0U;
      if ((msg->data[4] & 0x1U) != 0U) {
        tx = false;
      }
    } else {
      // LaRefAng_No_Req is a RELATIVE correction carrying the whole remaining error, which
      // the PSCM applies over its own internal ramp; it is not a target openpilot ramps, so
      // a per-frame rate-of-change bound would measure intent, not motion. This platform
      // gets its own check: the magnitude of the relative request, the ISO lateral
      // acceleration of the absolute target it reconstructs to, and the real-time cap on
      // commands per interval.
      const AngleSteeringParams FORD_LKA_STEERING_PARAMS = {
        .slip_factor = -0.0004472752575630534f,  // calc_slip_factor(VM) for FORD_TRANSIT_MK5
        .steer_ratio = 20.9,
        .wheelbase = 3.75,
      };
      // tenths of a degree, matching the StePinComp_An_Est angle_meas sample
      const float FORD_LKA_DEG_TO_CAN = 10.0f;
      // LaRefAng_No_Req is 12 bits at 0.05 mrad/bit with a -102.4 mrad offset: +/-5.9 deg,
      // i.e. +/-59 once rounded to tenths. Guards panda's own extraction, not the car.
      const int FORD_LKA_MAX_REL_ANGLE = 59;
      const float FORD_LKA_MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^2
      const AngleSteeringLimits FORD_LKA_RT_LIMITS = {
        .frequency = 33U,  // Lane_Assist_Data1 is sent at 33Hz
      };

      // Only the four intervention requests actuate steering: 1/6 increasing left/right,
      // 2/4 standard left/right. 0 is idle, 3/5 suppress LKA, 7 is NotUsed.
      bool steer_control_enabled = (action == 1U) || (action == 2U) || (action == 4U) || (action == 6U);
      bool valid_action = (action == 0U) || steer_control_enabled;

      // Signal: LaRefAng_No_Req : 19|12@0+ (0.05,-102.4) mrad, bits [19:8], RELATIVE to the
      // current pinion angle.
      unsigned int raw_rel = ((msg->data[2] & 0x0FU) << 8) | msg->data[3];
      float rel_mrad = ((float)raw_rel * 0.05f) - 102.4f;
      int rel_tenths = ROUND(rel_mrad * (1.8f / 3.14159265f));

      // The PSCM ignores the requested angle when the action is idle, and an all-zero
      // heartbeat payload decodes to -102.4 mrad, not zero. rel_tenths is not zeroed for
      // an idle frame: it is only ever read below inside `steer_control_enabled`, so the
      // idle frame carries no steering request because of that guard, not because the
      // value itself is cleared.

      // The only place the continuation latch is consulted. Every other branch of this
      // hook keeps gating on controls_allowed alone, which confines the latch to lateral.
      const bool lat_allowed = controls_allowed || ford_lka_continuation;

      bool violation = false;

      if (lat_allowed && steer_control_enabled) {
        // relative request magnitude
        violation |= safety_max_limit_check(rel_tenths, FORD_LKA_MAX_REL_ANGLE, -FORD_LKA_MAX_REL_ANGLE);

        // ISO lateral accel limit on the reconstructed absolute target: same fudged
        // speed, same vehicle model helpers and ceiling as steer_angle_cmd_checks_vm. This
        // is what stops a sustained maximum relative request winding the wheel up. Checked
        // against both ends of angle_meas's rolling window, matching this file's own
        // min/max convention (e.g. lateral.h's inactive-angle and angle-error bounds):
        // values[0] alone is just the latest sample and would skip the window extreme.
        const float fudged_speed = SAFETY_MAX((vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0, 1.0);
        const float curvature_factor = get_curvature_factor(fudged_speed, FORD_LKA_STEERING_PARAMS);
        const float max_curvature = FORD_LKA_MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed);
        const float max_angle = get_angle_from_curvature(max_curvature, curvature_factor, FORD_LKA_STEERING_PARAMS);
        const int max_angle_can = (int)((max_angle * FORD_LKA_DEG_TO_CAN) + 1.0f);
        violation |= safety_max_limit_check(angle_meas.min + rel_tenths, max_angle_can, -max_angle_can);
        violation |= safety_max_limit_check(angle_meas.max + rel_tenths, max_angle_can, -max_angle_can);

        // real time rate limit: the PSCM's own ramp was measured against a 33Hz stream
        violation |= rt_angle_rate_limit_check(FORD_LKA_RT_LIMITS);
      }

      // No steering request allowed when lateral control is not allowed
      violation |= !lat_allowed && steer_control_enabled;
      violation |= !valid_action;

      if (violation) {
        tx = false;
      }
    }
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);

    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.angle_deg_to_can to get real curvature
    int desired_curvature_rate = raw_curvature_rate - FORD_INACTIVE_CURVATURE_RATE;
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;

    bool violation = false;
    if (ford_lka_steering) {
      // LKA_STEERING platforms steer through Lane_Assist_Data1. This message stays
      // transmittable so the camera heartbeat survives, but it must never carry a steering
      // request, and the curvature angle check is not run: angle_meas holds the pinion
      // angle in tenths of a degree here, not a curvature.
      violation |= steer_control_enabled;
    } else if (ford_extended_lateral) {
      violation |= desired_path_offset != 0;
      violation |= (desired_curvature_rate < -4096) || (desired_curvature_rate > 4095);
      violation |= desired_path_angle != 0;
      violation |= steer_angle_cmd_checks(desired_curvature, steer_control_enabled,
                                          FORD_EXTENDED_STEERING_LIMITS);
      if (!steer_control_enabled) {
        violation |= (desired_curvature != 0) || (desired_curvature_rate != 0);
      }
    } else {
      violation |= (raw_curvature_rate != FORD_INACTIVE_CURVATURE_RATE) ||
                   (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) ||
                   (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);
      violation |= steer_angle_cmd_checks(desired_curvature, steer_control_enabled, FORD_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    static const AngleSteeringLimits FORD_CANFD_STEERING_LIMITS = FORD_LIMITS(true);
    static const AngleSteeringLimits FORD_CANFD_EXTENDED_STEERING_LIMITS = FORD_EXTENDED_LIMITS(true);

    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];

    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.angle_deg_to_can to get real curvature
    int desired_curvature_rate = raw_curvature_rate - FORD_CANFD_INACTIVE_CURVATURE_RATE;
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;

    bool violation = false;
    if (ford_lka_steering) {
      // LKA_STEERING platforms steer through Lane_Assist_Data1. This message stays
      // transmittable so the camera heartbeat survives, but it must never carry a steering
      // request, and the curvature angle check is not run: angle_meas holds the pinion
      // angle in tenths of a degree here, not a curvature.
      violation |= steer_control_enabled;
    } else if (ford_extended_lateral) {
      violation |= desired_path_offset != 0;
      violation |= (desired_curvature_rate < -1024) || (desired_curvature_rate > 1023);
      violation |= desired_path_angle != 0;
      violation |= steer_angle_cmd_checks(desired_curvature, steer_control_enabled,
                                          FORD_CANFD_EXTENDED_STEERING_LIMITS);
      if (!steer_control_enabled) {
        violation |= (desired_curvature != 0) || (desired_curvature_rate != 0);
      }
    } else {
      violation |= (raw_curvature_rate != FORD_CANFD_INACTIVE_CURVATURE_RATE) ||
                   (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) ||
                   (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);
      violation |= steer_angle_cmd_checks(desired_curvature, steer_control_enabled,
                                          FORD_CANFD_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  #define FORD_COMMON_RX_CHECKS \
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},                                        \
    /* FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug */                      \
    /* Some hybrid models also experience a bug where this checksum mismatches for one or two frames */                    \
    /* under heavy acceleration with ACC. The Bronco Sport's camera only disallows ACC for bad quality */                   \
    /* flags, not counters or checksums, so we match that */                                                               \
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},      \
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}},                                          \
    /* These messages have no counter or checksum */                                                                       \
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

  static RxCheck ford_rx_checks[] = {
    FORD_COMMON_RX_CHECKS
  };

  // LKA_STEERING platforms additionally require the measured pinion angle. Scoped to them
  // so a missing SteeringPinion_Data cannot disable controls on other Fords. The quality
  // flag is checked; the counter and checksum are not, because this PSCM does not transmit
  // them (constant zero across 856k captured frames), and enforcing the counter would
  // permanently disable controls.
  static RxCheck ford_lka_rx_checks[] = {
    FORD_COMMON_RX_CHECKS
    {.msg = {{FORD_SteeringPinion_Data, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
  };

  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const uint16_t FORD_PARAM_LKA_STEERING = 4;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);
  ford_lka_steering = GET_FLAG(param, FORD_PARAM_LKA_STEERING);
  ford_extended_lateral = false;
  ford_cancel_resume_button = false;

  // Lateral continuation, only on a platform that steers through Lane_Assist_Data1. The
  // latch itself always starts clear, so a mode change cannot inherit one.
  const uint16_t FORD_PARAM_LKA_CONTINUATION = 8;
  ford_lka_continuation_enabled = ford_lka_steering && GET_FLAG(param, FORD_PARAM_LKA_CONTINUATION);
  ford_lka_continuation = false;

  ford_longitudinal = false;

#ifdef ALLOW_DEBUG
  const uint16_t FORD_PARAM_LONGITUDINAL = 1;
  ford_longitudinal = GET_FLAG(param, FORD_PARAM_LONGITUDINAL);
#endif

  safety_config ret;
  if (ford_lka_steering) {
    if (ford_canfd) {
      ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_lka_rx_checks, FORD_CANFD_LONG_TX_MSGS) : \
                                BUILD_SAFETY_CFG(ford_lka_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
    } else {
      ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_lka_rx_checks, FORD_LONG_TX_MSGS) : \
                                BUILD_SAFETY_CFG(ford_lka_rx_checks, FORD_STOCK_TX_MSGS);
    }
  } else {
    if (ford_canfd) {
      ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS) : \
                                BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
    } else {
      ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS) : \
                                BUILD_SAFETY_CFG(ford_rx_checks, FORD_STOCK_TX_MSGS);
    }
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};
