/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_estimator_altitude.c: Altitude-only position estimator
 */

#include "stm32_legacy.h"
#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "num.h"
#include "position_estimator.h"
#define DEBUG_MODULE "POSEST"
#include "debug_cf.h"

#define G 9.81f;

struct selfState_s {
  float estimatedZ; // The current Z estimate, has same offset as asl
  float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlphaZrange;
  float estAlphaAsl;
  float velocityFactor;
  float vAccDeadband; // Vertical acceleration deadband
  float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
  float estimatedVZ;
  // pyDrone altitude-hold handling: the SPL06 sits on the PCB top and its reading
  // is corrupted by prop wash at low altitude. When altitude-hold engages we treat
  // the current position as 0, then run a TIME-BASED baro warm-up (ported from the
  // mp-new firmware): for warmupTicks the baro is ignored (pure accelerometer), then
  // its weight ramps in over softTicks. This avoids prop-wash pollution near the
  // ground without the stall of distance-based dead-reckoning.
  float lastAsl;        // most recent baro asl (m), used to capture the reference
  float aslRef;         // baro reference captured at engage (m)
  bool  holdEngaged;    // altitude-hold currently engaged
  uint16_t warmupCount; // ticks remaining where baro is ignored (pure acc)
  uint16_t softCount;   // ticks remaining of the baro weight soft-ramp
  uint16_t warmupTicks; // warm-up length (ticks @100Hz). 90 = 0.9s
  uint16_t softTicks;   // soft-ramp length (ticks @100Hz). 40 = 0.4s
};

static struct selfState_s state = {
  .estimatedZ = 0.0f,
  .velocityZ = 0.0f,
  .estAlphaZrange = 0.90f,
  // pyDrone: the SPL06 reads cleanly and quickly, but the original 0.997 (~3.3s
  // time constant at 100Hz) made the fused z lag the baro by several seconds.
  // 0.95 (~0.2s) tracks the baro fast. Tune live via posEstAlt.estAlphaAsl:
  // raise it (toward 0.99) if flight altitude is jittery from prop wash,
  // lower it if altitude response feels laggy.
  .estAlphaAsl = 0.95f,
  .velocityFactor = 1.0f,
  .vAccDeadband = 0.04f,
  .velZAlpha = 0.995f,
  .estimatedVZ = 0.0f,
  .lastAsl = 0.0f,
  .aslRef = 0.0f,
  .holdEngaged = false,
  .warmupCount = 0,
  .softCount = 0,
  .warmupTicks = 90,   // 0.9s @100Hz: ignore baro after engage (prop-wash window)
  .softTicks = 40,     // 0.4s @100Hz: ramp baro weight back in
};

static void positionEstimateInternal(state_t* estimate, const sensorData_t* sensorData, const tofMeasurement_t* tofMeasurement, float dt, uint32_t tick, struct selfState_s* state);
static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state);

void positionEstimate(state_t* estimate, const sensorData_t* sensorData, const tofMeasurement_t* tofMeasurement, float dt, uint32_t tick) {
  positionEstimateInternal(estimate, sensorData, tofMeasurement, dt, tick, &state);
}

void positionUpdateVelocity(float accWZ, float dt) {
  positionUpdateVelocityInternal(accWZ, dt, &state);
}

// Called when altitude-hold is engaged/disengaged (z setpoint mode active or not).
// On the engage edge: treat the current position as 0 and start dead-reckoning the
// climb from the accelerometer (the baro is unreliable near the ground).
void positionEstimatorAltitudeSetHoldEngaged(bool engaged) {
  if (engaged && !state.holdEngaged) {
    // Treat the current position as 0 and start the baro warm-up: ignore the baro
    // for warmupTicks (pure accelerometer), then ramp its weight back in.
    state.holdEngaged = true;
    state.estimatedZ = 0.0f;
    state.velocityZ = 0.0f;
    state.aslRef = state.lastAsl;
    state.warmupCount = state.warmupTicks;
    state.softCount = 0;
    DEBUG_PRINTI("ALT-HOLD engaged: zero here (aslRef=%.2f), baro warm-up", (double)state.aslRef);
  } else if (!engaged && state.holdEngaged) {
    state.holdEngaged = false;
    state.warmupCount = 0;
    state.softCount = 0;
    DEBUG_PRINTI("ALT-HOLD disengaged");
  }
}

static void positionEstimateInternal(state_t* estimate, const sensorData_t* sensorData, const tofMeasurement_t* tofMeasurement, float dt, uint32_t tick, struct selfState_s* state) {
  float filteredZ;
  static float prev_estimatedZ = 0;
  static bool surfaceFollowingMode = false;

  const uint32_t MAX_SAMPLE_AGE = M2T(50);

  uint32_t now = xTaskGetTickCount();
  bool isSampleUseful = ((now - tofMeasurement->timestamp) <= MAX_SAMPLE_AGE);

  if (isSampleUseful) {
    surfaceFollowingMode = true;
  }

  if (surfaceFollowingMode) {
    if (isSampleUseful) {
      // IIR filter zrange
      filteredZ = (state->estAlphaZrange       ) * state->estimatedZ +
                  (1.0f - state->estAlphaZrange) * tofMeasurement->distance;
      // Use zrange as base and add velocity changes.
      state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);
    }
  } else {
    state->lastAsl = sensorData->baro.asl;

    if (!state->holdEngaged) {
      // Not in altitude-hold: just track baro for display (not used for control).
      // FIXME: A bit of an hack to init IIR filter
      if (state->estimatedZ == 0.0f) {
        filteredZ = sensorData->baro.asl;
      } else {
        // IIR filter asl
        filteredZ = (state->estAlphaAsl       ) * state->estimatedZ +
                    (1.0f - state->estAlphaAsl) * sensorData->baro.asl;
      }
      state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);
    } else {
      // Engaged: hold altitude RELATIVE to the engage point. Complementary filter
      // of baro (absolute reference) + acc velocity (fast response), but the baro
      // WEIGHT is staged to dodge prop-wash pollution near the ground (ported from
      // mp-new), and is TIME-based (not distance-based) so it can't stall:
      //   warm-up : baro weight 0 -> pure accelerometer for warmupTicks
      //   soft    : weight ramps 0 -> (1-estAlphaAsl) over softTicks
      //   normal  : weight = (1-estAlphaAsl)
      float baroRel = sensorData->baro.asl - state->aslRef;
      float baroW;
      if (state->warmupCount > 0) {
        state->warmupCount--;
        baroW = 0.0f;                                   // ignore baro (prop-wash window)
        if (state->warmupCount == 0) {
          state->softCount = state->softTicks;          // start ramping baro back in
        }
      } else if (state->softCount > 0) {
        state->softCount--;
        float ramp = 1.0f - (float)state->softCount / (float)state->softTicks; // 0 -> 1
        baroW = (1.0f - state->estAlphaAsl) * ramp;
      } else {
        baroW = (1.0f - state->estAlphaAsl);            // steady state
      }
      filteredZ = (1.0f - baroW) * state->estimatedZ + baroW * baroRel;
      state->estimatedZ = filteredZ + (state->velocityFactor * state->velocityZ * dt);
    }
  }

  estimate->position.x = 0.0f;
  estimate->position.y = 0.0f;
  estimate->position.z = state->estimatedZ;
  estimate->velocity.z = (state->estimatedZ - prev_estimatedZ) / dt;
  state->estimatedVZ = estimate->velocity.z;
  prev_estimatedZ = state->estimatedZ;
}

static void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state) {
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * G;
  state->velocityZ *= state->velZAlpha;
}

LOG_GROUP_START(posEstAlt)
LOG_ADD(LOG_FLOAT, estimatedZ, &state.estimatedZ)
LOG_ADD(LOG_FLOAT, estVZ, &state.estimatedVZ)
LOG_ADD(LOG_FLOAT, velocityZ, &state.velocityZ)
LOG_ADD(LOG_UINT16, warmup, &state.warmupCount)
LOG_ADD(LOG_UINT8, holdEng, &state.holdEngaged)
LOG_GROUP_STOP(posEstAlt)

PARAM_GROUP_START(posEstAlt)
PARAM_ADD(PARAM_FLOAT, estAlphaAsl, &state.estAlphaAsl)
PARAM_ADD(PARAM_FLOAT, estAlphaZr, &state.estAlphaZrange)
PARAM_ADD(PARAM_FLOAT, velFactor, &state.velocityFactor)
PARAM_ADD(PARAM_FLOAT, velZAlpha, &state.velZAlpha)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &state.vAccDeadband)
PARAM_ADD(PARAM_UINT16, warmupTicks, &state.warmupTicks)
PARAM_ADD(PARAM_UINT16, softTicks, &state.softTicks)
PARAM_GROUP_STOP(posEstAlt)
