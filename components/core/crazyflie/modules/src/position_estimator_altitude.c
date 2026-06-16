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
  // is corrupted by prop wash at low altitude. When altitude-hold is engaged we
  // treat the current position as 0 and dead-reckon height from the accelerometer
  // until we climb past baroHandoffM, then hand over to the (now reliable) baro.
  float lastAsl;       // most recent baro asl (m), used to capture the reference
  float aslRef;        // baro reference captured at engage / handoff (m)
  float baroHandoffM;  // climb height at which we switch acc -> baro (m)
  bool  holdEngaged;   // altitude-hold currently engaged
  bool  accClimbMode;  // dead-reckoning the initial climb from the accelerometer
};

static struct selfState_s state = {
  .estimatedZ = 0.0f,
  .velocityZ = 0.0f,
  .estAlphaZrange = 0.90f,
  .estAlphaAsl = 0.997f,
  .velocityFactor = 1.0f,
  .vAccDeadband = 0.04f,
  .velZAlpha = 0.995f,
  .estimatedVZ = 0.0f,
  .lastAsl = 0.0f,
  .aslRef = 0.0f,
  .baroHandoffM = 0.6f,
  .holdEngaged = false,
  .accClimbMode = false,
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
    state.holdEngaged = true;
    state.accClimbMode = true;
    state.estimatedZ = 0.0f;
    state.velocityZ = 0.0f;
    state.aslRef = state.lastAsl;
  } else if (!engaged && state.holdEngaged) {
    state.holdEngaged = false;
    state.accClimbMode = false;
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
    } else if (state->accClimbMode) {
      // Engaged + low altitude: baro corrupted by prop wash, dead-reckon from acc.
      state->estimatedZ = state->estimatedZ + (state->velocityFactor * state->velocityZ * dt);
      if (state->estimatedZ >= state->baroHandoffM) {
        // Hand over to baro without a step: reference it to the current estimate.
        state->accClimbMode = false;
        state->aslRef = sensorData->baro.asl - state->estimatedZ;
      }
    } else {
      // Engaged + above handoff: baro relative to the engage reference + acc velocity.
      float baroRel = sensorData->baro.asl - state->aslRef;
      filteredZ = (state->estAlphaAsl       ) * state->estimatedZ +
                  (1.0f - state->estAlphaAsl) * baroRel;
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
LOG_ADD(LOG_UINT8, accClimb, &state.accClimbMode)
LOG_ADD(LOG_UINT8, holdEng, &state.holdEngaged)
LOG_GROUP_STOP(posEstAlt)

PARAM_GROUP_START(posEstAlt)
PARAM_ADD(PARAM_FLOAT, estAlphaAsl, &state.estAlphaAsl)
PARAM_ADD(PARAM_FLOAT, estAlphaZr, &state.estAlphaZrange)
PARAM_ADD(PARAM_FLOAT, velFactor, &state.velocityFactor)
PARAM_ADD(PARAM_FLOAT, velZAlpha, &state.velZAlpha)
PARAM_ADD(PARAM_FLOAT, vAccDeadband, &state.vAccDeadband)
PARAM_ADD(PARAM_FLOAT, baroHandoffM, &state.baroHandoffM)
PARAM_GROUP_STOP(posEstAlt)
