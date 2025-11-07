/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * attitude_controller.h: PID-based attitude controller
 */

#ifndef FUZZY_CONTROLLER_H_
#define FUZZY_CONTROLLER_H_

#include "stabilizer_types.h"

#include <stdbool.h>
#include <stdint.h>

// A position controller calculate the thrust, roll, pitch to approach
// a 3D position setpoint
void fuzzyControllerInit(const float updateDt);
bool fuzzyControllerTest(void);

void fuzzypositionController(float* thrust, attitude_t *attitude, const setpoint_t *setpoint, const state_t *state);
void fuzzyvelocityController(float* thrust, attitude_t *attitude, const Axis3f *setpoint_velocity, const state_t *state);

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 */
void attitudeControllerCorrectAttitudefuzzy(
        float eulerRollActual, float eulerPitchActual, float eulerYawActual,
        float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
        float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

/**
 * Make the controller run an update of the rate PID. The output is
 * the actuator force.
 */
void attitudeControllerCorrectRatefuzzy(
        float rollRateActual, float pitchRateActual, float yawRateActual,
        float rollRateDesired, float pitchRateDesired, float yawRateDesired);
       
/**
 * Get the actuator output.
 */
void fuzzyattitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

/**
 * Get yaw max delta
 */
float fuzzyattitudeControllerGetYawMaxDelta(void);


// reset 
/**
 * @brief Resets all position and velocity PID controllers (P, I, D accumulators).
 */
void positionControllerResetAllfuzzy(float xActual, float yActual, float zActual);

/**
 * @brief Resets all input/target filters for the position and velocity PID controllers.
 */
void fuzzypositionControllerResetAllfilters();

/**
 * @brief Resets roll attitude PID.
 */
void attitudeControllerResetRollAttitudefuzzy(float rollActual);

/**
 * @brief Resets pitch attitude PID.
 */
void attitudeControllerResetPitchAttitudefuzzy(float pitchActual);

/**
 * @brief Resets all attitude (Roll, Pitch, Yaw) and rate PID controllers.
 */
void attitudeControllerResetAllfuzzy(float rollActual, float pitchActual, float yawActual);

#endif /* FUZZY_CONTROLLER_H_ */
