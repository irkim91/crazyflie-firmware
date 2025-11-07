/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 * attitude_pid_controller.c: Attitude controller using PID correctors
 */
#include <stdbool.h>
#include <math.h>

#include "stabilizer_types.h"

#include "fuzzy_controller.h"
#include "pid.h"
#include "param.h"
#include "num.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"

#define DT (float)(1.0f/POSITION_RATE)

typedef struct {
  PidObject pid;

  stab_mode_t previousMode;
  float setpoint;
  float output;
} PidAxis_t;

typedef struct {
  // Position Loop: m -> m/s
  PidAxis_t pidX;
  PidAxis_t pidY;
  PidAxis_t pidZ;

  // Velocity Loop: m/s -> angle/thrust
  PidAxis_t pidVX; 
  PidAxis_t pidVY; 
  PidAxis_t pidVZ; 

  // Attitude Loop: angle -> rate
  PidAxis_t pidRoll;
  PidAxis_t pidPitch;
  PidAxis_t pidYaw;

  // Rate Loop: rate -> motor power
  PidAxis_t pidRollRate;
  PidAxis_t pidPitchRate;
  PidAxis_t pidYawRate;

  //
  uint16_t thrustBase;
  uint16_t thrustMin;
  bool isInit; 

  int16_t rollOutput;
  int16_t pitchOutput;
  int16_t yawOutput;

} ControllerState_t;

// Maximum roll/pitch angle permited
static float rLimit = PID_VEL_ROLL_MAX;
static float pLimit = PID_VEL_PITCH_MAX;
static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xVelMax = PID_POS_VEL_X_MAX;
static float yVelMax = PID_POS_VEL_Y_MAX;
static float zVelMax = PID_POS_VEL_Z_MAX;
static float velMaxOverhead = 1.10f;
static const float thrustScale = 1000.0f;

// pos & vel
static bool posFiltEnable = PID_POS_XY_FILT_ENABLE;
static bool velFiltEnable = PID_VEL_XY_FILT_ENABLE;
static float posFiltCutoff = PID_POS_XY_FILT_CUTOFF;
static float velFiltCutoff = PID_VEL_XY_FILT_CUTOFF;
static bool posZFiltEnable = PID_POS_Z_FILT_ENABLE;
static bool velZFiltEnable = PID_VEL_Z_FILT_ENABLE;
static float posZFiltCutoff = PID_POS_Z_FILT_CUTOFF;
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF;

// att & rate
static bool attFiltEnable   = ATTITUDE_LPF_ENABLE;
static bool rateFiltEnable  = ATTITUDE_RATE_LPF_ENABLE;
static float attFiltCutoff  = ATTITUDE_LPF_CUTOFF_FREQ;
static float omxFiltCutoff  = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
static float omyFiltCutoff  = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
static float omzFiltCutoff  = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;
static float yawMaxDelta    = YAW_MAX_DELTA;

static ControllerState_t CState = {

  .pidVX = { .pid = { .kp = PID_VEL_X_KP, .ki = PID_VEL_X_KI, .kd = PID_VEL_X_KD, .kff = PID_VEL_X_KFF, .dt = DT } },
  .pidVY = { .pid = { .kp = PID_VEL_Y_KP, .ki = PID_VEL_Y_KI, .kd = PID_VEL_Y_KD, .kff = PID_VEL_Y_KFF, .dt = DT } },
  .pidVZ = { .pid = { .kp = PID_VEL_Z_KP, .ki = PID_VEL_Z_KI, .kd = PID_VEL_Z_KD, .kff = PID_VEL_Z_KFF, .dt = DT } },

  .pidX = { .pid = { .kp = PID_POS_X_KP, .ki = PID_POS_X_KI, .kd = PID_POS_X_KD, .kff = PID_POS_X_KFF, .dt = DT } },
  .pidY = { .pid = { .kp = PID_POS_Y_KP, .ki = PID_POS_Y_KI, .kd = PID_POS_Y_KD, .kff = PID_POS_Y_KFF, .dt = DT } },
  .pidZ = { .pid = { .kp = PID_POS_Z_KP, .ki = PID_POS_Z_KI, .kd = PID_POS_Z_KD, .kff = PID_POS_Z_KFF, .dt = DT } },

  .pidRollRate  = { .pid = { .kp = PID_ROLL_RATE_KP, .ki = PID_ROLL_RATE_KI, .kd = PID_ROLL_RATE_KD, .kff = PID_ROLL_RATE_KFF } },
  .pidPitchRate = { .pid = { .kp = PID_PITCH_RATE_KP, .ki = PID_PITCH_RATE_KI, .kd = PID_PITCH_RATE_KD, .kff = PID_PITCH_RATE_KFF } },
  .pidYawRate   = { .pid = { .kp = PID_YAW_RATE_KP, .ki = PID_YAW_RATE_KI, .kd = PID_YAW_RATE_KD, .kff = PID_YAW_RATE_KFF } },

  .pidRoll  = { .pid = { .kp = PID_ROLL_KP, .ki = PID_ROLL_KI, .kd = PID_ROLL_KD, .kff = PID_ROLL_KFF } },
  .pidPitch = { .pid = { .kp = PID_PITCH_KP, .ki = PID_PITCH_KI, .kd = PID_PITCH_KD, .kff = PID_PITCH_KFF } },
  .pidYaw   = { .pid = { .kp = PID_YAW_KP, .ki = PID_YAW_KI, .kd = PID_YAW_KD, .kff = PID_YAW_KFF } },

  // 
  .thrustBase = PID_VEL_THRUST_BASE,
  .thrustMin  = PID_VEL_THRUST_MIN,

  .isInit     = false, 
  .rollOutput = 0,
  .pitchOutput = 0,
  .yawOutput  = 0,
};

float state_body_x, state_body_y, state_body_vx, state_body_vy;

static inline int16_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

static inline float runPid(float input, PidAxis_t *axis, float setpoint) {
  axis->setpoint = setpoint;

  pidSetDesired(&axis->pid, axis->setpoint);
  return pidUpdate(&axis->pid, input, false);
}

void fuzzyControllerInit(const float updateDt)
{
  if (CState.isInit) {
    return;
  }

  // Position
  pidInit(&CState.pidX.pid, CState.pidX.setpoint, CState.pidX.pid.kp, CState.pidX.pid.ki, CState.pidX.pid.kd, 
    CState.pidX.pid.kff, CState.pidX.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&CState.pidY.pid, CState.pidY.setpoint, CState.pidY.pid.kp, CState.pidY.pid.ki, CState.pidY.pid.kd,
          CState.pidY.pid.kff, CState.pidY.pid.dt, POSITION_RATE, posFiltCutoff, posFiltEnable);
  pidInit(&CState.pidZ.pid, CState.pidZ.setpoint, CState.pidZ.pid.kp, CState.pidZ.pid.ki, CState.pidZ.pid.kd,
          CState.pidZ.pid.kff, CState.pidZ.pid.dt, POSITION_RATE, posZFiltCutoff, posZFiltEnable);

  // Velocity
  pidInit(&CState.pidVX.pid, CState.pidVX.setpoint, CState.pidVX.pid.kp, CState.pidVX.pid.ki, CState.pidVX.pid.kd,
          CState.pidVX.pid.kff, CState.pidVX.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&CState.pidVY.pid, CState.pidVY.setpoint, CState.pidVY.pid.kp, CState.pidVY.pid.ki, CState.pidVY.pid.kd,
          CState.pidVY.pid.kff, CState.pidVY.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&CState.pidVZ.pid, CState.pidVZ.setpoint, CState.pidVZ.pid.kp, CState.pidVZ.pid.ki, CState.pidVZ.pid.kd,
          CState.pidVZ.pid.kff, CState.pidVZ.pid.dt, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
    
  // Rate 
  pidInit(&CState.pidRollRate.pid,  0, CState.pidRollRate.pid.kp, CState.pidRollRate.pid.ki, CState.pidRollRate.pid.kd,
          CState.pidRollRate.pid.kff, updateDt, ATTITUDE_RATE, omxFiltCutoff, rateFiltEnable);
  pidInit(&CState.pidPitchRate.pid, 0, CState.pidPitchRate.pid.kp, CState.pidPitchRate.pid.ki, CState.pidPitchRate.pid.kd,
          CState.pidPitchRate.pid.kff, updateDt, ATTITUDE_RATE, omyFiltCutoff, rateFiltEnable);
  pidInit(&CState.pidYawRate.pid,   0, CState.pidYawRate.pid.kp, CState.pidYawRate.pid.ki, CState.pidYawRate.pid.kd,
          CState.pidYawRate.pid.kff, updateDt, ATTITUDE_RATE, omzFiltCutoff, rateFiltEnable);

  pidSetIntegralLimit(&CState.pidRollRate.pid, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&CState.pidPitchRate.pid, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&CState.pidYawRate.pid,  PID_YAW_RATE_INTEGRATION_LIMIT);

  // Attitude 
  pidInit(&CState.pidRoll.pid, 0, CState.pidRoll.pid.kp, CState.pidRoll.pid.ki, CState.pidRoll.pid.kd, CState.pidRoll.pid.kff, updateDt,
          ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&CState.pidPitch.pid, 0, CState.pidPitch.pid.kp, CState.pidPitch.pid.ki, CState.pidPitch.pid.kd, CState.pidPitch.pid.kff, updateDt,
          ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&CState.pidYaw.pid, 0, CState.pidYaw.pid.kp, CState.pidYaw.pid.ki, CState.pidYaw.pid.kd, CState.pidYaw.pid.kff, updateDt,
          ATTITUDE_RATE, attFiltCutoff, attFiltEnable);

  pidSetIntegralLimit(&CState.pidRoll.pid,  PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&CState.pidPitch.pid, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&CState.pidYaw.pid,   PID_YAW_INTEGRATION_LIMIT);

  CState.isInit = true;
}

bool fuzzyControllerTest()
{
  return CState.isInit;
}

void fuzzypositionController(float* thrust, attitude_t *attitude, const setpoint_t *setpoint,
                                                             const state_t *state)
{
  CState.pidX.pid.outputLimit = xVelMax * velMaxOverhead;
  CState.pidY.pid.outputLimit = yVelMax * velMaxOverhead;
  // The ROS landing detector will prematurely trip if
  // this value is below 0.5
  CState.pidZ.pid.outputLimit = fmaxf(zVelMax, 0.5f) * velMaxOverhead;

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);

  float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
  float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;

  state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
  state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;

  float globalvx = setpoint->velocity.x;
  float globalvy = setpoint->velocity.y;

  //X, Y
  Axis3f setpoint_velocity;
  setpoint_velocity.x = setpoint->velocity.x;
  setpoint_velocity.y = setpoint->velocity.y;
  setpoint_velocity.z = setpoint->velocity.z;
  
  if (setpoint->mode.x == modeAbs) {
    setpoint_velocity.x = runPid(state_body_x, &CState.pidX, setp_body_x);
  } else if (!setpoint->velocity_body) {
    setpoint_velocity.x = globalvx * cosyaw + globalvy * sinyaw;
  }
  if (setpoint->mode.y == modeAbs) {
    setpoint_velocity.y = runPid(state_body_y, &CState.pidY, setp_body_y);
  } else if (!setpoint->velocity_body) {
    setpoint_velocity.y = globalvy * cosyaw - globalvx * sinyaw;
  }
  if (setpoint->mode.z == modeAbs) {
    setpoint_velocity.z = runPid(state->position.z, &CState.pidZ, setpoint->position.z);
  }

  fuzzyvelocityController(thrust, attitude, &setpoint_velocity, state);
}

void fuzzyvelocityController(float* thrust, attitude_t *attitude, const Axis3f* setpoint_velocity,
                                                             const state_t *state)
{
  CState.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  CState.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  // Set the output limit to the maximum thrust range
  CState.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //CState.pidVZ.pid.outputLimit = (CState.thrustBase - CState.thrustMin) / thrustScale; 

  float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
  float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
  state_body_vx = state->velocity.x * cosyaw + state->velocity.y * sinyaw;
  state_body_vy = -state->velocity.x * sinyaw + state->velocity.y * cosyaw;

  // Roll and Pitch
  attitude->pitch = -runPid(state_body_vx, &CState.pidVX, setpoint_velocity->x);
  attitude->roll  = -runPid(state_body_vy, &CState.pidVY, setpoint_velocity->y);

  attitude->roll  = constrain(attitude->roll,  -rLimit, rLimit);
  attitude->pitch = constrain(attitude->pitch, -pLimit, pLimit);

  // Thrust
  float thrustRaw = runPid(state->velocity.z, &CState.pidVZ, setpoint_velocity->z);
  // Scale the thrust and add feed forward term
  *thrust = thrustRaw * thrustScale + CState.thrustBase;  
  // Check for minimum thrust (CState 사용)
  if (*thrust < CState.thrustMin) {
    *thrust = CState.thrustMin;
  }
    // saturate
  *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void attitudeControllerCorrectRatefuzzy(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired)
{
  pidSetDesired(&CState.pidRollRate.pid, rollRateDesired);
  CState.rollOutput = saturateSignedInt16(pidUpdate(&CState.pidRollRate.pid, rollRateActual, false));

  pidSetDesired(&CState.pidPitchRate.pid, pitchRateDesired);
  CState.pitchOutput = saturateSignedInt16(pidUpdate(&CState.pidPitchRate.pid, pitchRateActual, false));

  pidSetDesired(&CState.pidYawRate.pid, yawRateDesired);
  CState.yawOutput = saturateSignedInt16(pidUpdate(&CState.pidYawRate.pid, yawRateActual, false));
}

void attitudeControllerCorrectAttitudefuzzy(
       float eulerRollActual,   float eulerPitchActual,   float eulerYawActual,
       float eulerRollDesired,  float eulerPitchDesired,  float eulerYawDesired,
       float* rollRateDesired,  float* pitchRateDesired,  float* yawRateDesired)
{
  pidSetDesired(&CState.pidRoll.pid, eulerRollDesired);
  *rollRateDesired = pidUpdate(&CState.pidRoll.pid, eulerRollActual, false);

  // Update PID for pitch axis
  pidSetDesired(&CState.pidPitch.pid, eulerPitchDesired);
  *pitchRateDesired = pidUpdate(&CState.pidPitch.pid, eulerPitchActual, false);

  // Update PID for yaw axis
  pidSetDesired(&CState.pidYaw.pid, eulerYawDesired);
  *yawRateDesired = pidUpdate(&CState.pidYaw.pid, eulerYawActual, true);
}

void positionControllerResetAllfuzzy(float xActual, float yActual, float zActual)
{
  pidReset(&CState.pidX.pid, xActual);
  pidReset(&CState.pidY.pid, yActual);
  pidReset(&CState.pidZ.pid, zActual);
  pidReset(&CState.pidVX.pid, 0);
  pidReset(&CState.pidVY.pid, 0);
  pidReset(&CState.pidVZ.pid, 0);
}

void fuzzypositionControllerResetAllfilters() {
  filterReset(&CState.pidX.pid, POSITION_RATE, posFiltCutoff, posFiltEnable);
  filterReset(&CState.pidY.pid, POSITION_RATE, posFiltCutoff, posFiltEnable);
  filterReset(&CState.pidZ.pid, POSITION_RATE, posZFiltCutoff, posZFiltEnable);
  filterReset(&CState.pidVX.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&CState.pidVY.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&CState.pidVZ.pid, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}

void attitudeControllerResetRollAttitudefuzzy(float rollActual)
{
  pidReset(&CState.pidRoll.pid, rollActual);
}

void attitudeControllerResetPitchAttitudefuzzy(float pitchActual)
{
  pidReset(&CState.pidPitch.pid, pitchActual);
}

void attitudeControllerResetAllfuzzy(float rollActual, float pitchActual, float yawActual)
{
  pidReset(&CState.pidRoll.pid, rollActual);
  pidReset(&CState.pidPitch.pid, pitchActual);
  pidReset(&CState.pidYaw.pid, yawActual);
  pidReset(&CState.pidRollRate.pid, 0);
  pidReset(&CState.pidPitchRate.pid, 0);
  pidReset(&CState.pidYawRate.pid, 0);
}

void fuzzyattitudeControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw)
{
  *roll   = CState.rollOutput;
  *pitch  = CState.pitchOutput;
  *yaw    = CState.yawOutput;
}

float fuzzyattitudeControllerGetYawMaxDelta(void)
{
  return yawMaxDelta;
}

LOG_GROUP_START(posCtl)

/**
 * @brief PID controller target desired body-yaw-aligned velocity x [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVX, &CState.pidVX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned velocity y [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVY, &CState.pidVY.pid.desired)
/**
 * @brief PID controller target desired velocity z [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVZ, &CState.pidVZ.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position x [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetX, &CState.pidX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position y [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetY, &CState.pidY.pid.desired)
/**
 * @brief PID controller target desired global position z [m]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetZ, &CState.pidZ.pid.desired)

/**
 * @brief PID state body-yaw-aligned velocity x [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVX, &state_body_vx)
/**
 * @brief PID state body-yaw-aligned velocity y [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVY, &state_body_vy)
/**
 * @brief PID state body-yaw-aligned position x [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyX, &state_body_x)
/**
 * @brief PID state body-yaw-aligned position y [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyY, &state_body_y)

/**
 * @brief PID proportional output position x
 */
LOG_ADD(LOG_FLOAT, Xp, &CState.pidX.pid.outP)
/**
 * @brief PID integral output position x
 */
LOG_ADD(LOG_FLOAT, Xi, &CState.pidX.pid.outI)
/**
 * @brief PID derivative output position x
 */
LOG_ADD(LOG_FLOAT, Xd, &CState.pidX.pid.outD)
/**
 * @brief PID feedforward output position x
 */
LOG_ADD(LOG_FLOAT, Xff, &CState.pidX.pid.outFF)

/**
 * @brief PID proportional output position y
 */
LOG_ADD(LOG_FLOAT, Yp, &CState.pidY.pid.outP)
/**
 * @brief PID integral output position y
 */
LOG_ADD(LOG_FLOAT, Yi, &CState.pidY.pid.outI)
/**
 * @brief PID derivative output position y
 */
LOG_ADD(LOG_FLOAT, Yd, &CState.pidY.pid.outD)
/**
 * @brief PID feedforward output position y
 */
LOG_ADD(LOG_FLOAT, Yff, &CState.pidY.pid.outFF)

/**
 * @brief PID proportional output position z
 */
LOG_ADD(LOG_FLOAT, Zp, &CState.pidZ.pid.outP)
/**
 * @brief PID integral output position z
 */
LOG_ADD(LOG_FLOAT, Zi, &CState.pidZ.pid.outI)
/**
 * @brief PID derivative output position z
 */
LOG_ADD(LOG_FLOAT, Zd, &CState.pidZ.pid.outD)
/**
 * @brief PID feedforward output position z
 */
LOG_ADD(LOG_FLOAT, Zff, &CState.pidZ.pid.outFF)

/**
 * @brief PID proportional output velocity x
 */
LOG_ADD(LOG_FLOAT, VXp, &CState.pidVX.pid.outP)
/**
 * @brief PID integral output velocity x
 */
LOG_ADD(LOG_FLOAT, VXi, &CState.pidVX.pid.outI)
/**
 * @brief PID derivative output velocity x
 */
LOG_ADD(LOG_FLOAT, VXd, &CState.pidVX.pid.outD)
/**
 * @brief PID feedforward output velocity x
 */
LOG_ADD(LOG_FLOAT, VXff, &CState.pidVX.pid.outFF)

/**
 * @brief PID proportional output velocity y
 */
LOG_ADD(LOG_FLOAT, VYp, &CState.pidVY.pid.outP)
/**
 * @brief PID integral output velocity y
 */
LOG_ADD(LOG_FLOAT, VYi, &CState.pidVY.pid.outI)
/**
 * @brief PID derivative output velocity y
 */
LOG_ADD(LOG_FLOAT, VYd, &CState.pidVY.pid.outD)
/**
 * @brief PID feedforward output velocity y
 */
LOG_ADD(LOG_FLOAT, VYff, &CState.pidVY.pid.outFF)

/**
 * @brief PID proportional output velocity z
 */
LOG_ADD(LOG_FLOAT, VZp, &CState.pidVZ.pid.outP)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZi, &CState.pidVZ.pid.outI)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZd, &CState.pidVZ.pid.outD)
/**
 * @brief PID feedforward output velocity z
 */
LOG_ADD(LOG_FLOAT, VZff, &CState.pidVZ.pid.outFF)

LOG_GROUP_STOP(posCtl)

LOG_GROUP_START(pid_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &CState.pidRoll.pid.outP)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &CState.pidRoll.pid.outI)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &CState.pidRoll.pid.outD)
/**
 * @brief Feedforward output roll
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &CState.pidRoll.pid.outFF)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &CState.pidPitch.pid.outP)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &CState.pidPitch.pid.outI)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &CState.pidPitch.pid.outD)
/**
 * @brief Feedforward output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &CState.pidPitch.pid.outFF)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &CState.pidYaw.pid.outP)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &CState.pidYaw.pid.outI)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &CState.pidYaw.pid.outD)
/**
 * @brief Feedforward output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &CState.pidYaw.pid.outFF)
LOG_GROUP_STOP(pid_attitude)

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &CState.pidRollRate.pid.outP)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &CState.pidRollRate.pid.outI)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &CState.pidRollRate.pid.outD)
/**
 * @brief Feedforward output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outFF, &CState.pidRollRate.pid.outFF)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &CState.pidPitchRate.pid.outP)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &CState.pidPitchRate.pid.outI)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &CState.pidPitchRate.pid.outD)
/**
 * @brief Feedforward output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outFF, &CState.pidPitchRate.pid.outFF)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &CState.pidYawRate.pid.outP)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &CState.pidYawRate.pid.outI)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &CState.pidYawRate.pid.outD)
/**
 * @brief Feedforward output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outFF, &CState.pidYawRate.pid.outFF)
LOG_GROUP_STOP(pid_rate)

PARAM_GROUP_START(velCtlPid)
/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKp, &CState.pidVX.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKi, &CState.pidVX.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKd, &CState.pidVX.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the body-yaw-aligned X direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKFF, &CState.pidVX.pid.kff)

/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKp, &CState.pidVY.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKi, &CState.pidVY.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKd, &CState.pidVY.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the body-yaw-aligned Y direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKFF, &CState.pidVY.pid.kff)

/**
 * @brief Proportional gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKp, &CState.pidVZ.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKi, &CState.pidVZ.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKd, &CState.pidVZ.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the global direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKFF, &CState.pidVZ.pid.kff)

PARAM_GROUP_STOP(velCtlPid)

/**
 * Tuning settings for the gains of the PID
 * controller for the position of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(posCtlPid)
/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKp, &CState.pidX.pid.kp)
/**
 * @brief Integral gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKi, &CState.pidX.pid.ki)
/**
 * @brief Derivative gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKd, &CState.pidX.pid.kd)
/**
 * @brief Feedforward gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xKff, &CState.pidX.pid.kff)

/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKp, &CState.pidY.pid.kp)
/**
 * @brief Integral gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKi, &CState.pidY.pid.ki)
/**
 * @brief Derivative gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKd, &CState.pidY.pid.kd)
/**
 * @brief Feedforward gain for the position PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yKff, &CState.pidY.pid.kff)

/**
 * @brief Proportional gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKp, &CState.pidZ.pid.kp)
/**
 * @brief Integral gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKi, &CState.pidZ.pid.ki)
/**
 * @brief Derivative gain for the position PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKd, &CState.pidZ.pid.kd)
/**
 * @brief Feedforward gain for the position PID in the body-yaw-aligned Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zKff, &CState.pidZ.pid.kff)

/**
 * @brief Approx. thrust needed for hover
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, thrustBase, &CState.thrustBase)
/**
 * @brief Min. thrust value to output
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, thrustMin, &CState.thrustMin)

/**
 * @brief Roll absolute limit
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rLimit,  &rLimit)
/**
 * @brief Pitch absolute limit
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pLimit,  &pLimit)
/**
 * @brief Maximum body-yaw-aligned X velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xVelMax, &xVelMax)
/**
 * @brief Maximum body-yaw-aligned Y velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yVelMax, &yVelMax)
/**
 * @brief Maximum Z Velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlPid)

PARAM_GROUP_START(pid_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &CState.pidRoll.pid.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &CState.pidRoll.pid.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &CState.pidRoll.pid.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &CState.pidRoll.pid.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &CState.pidPitch.pid.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &CState.pidPitch.pid.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &CState.pidPitch.pid.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &CState.pidPitch.pid.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &CState.pidYaw.pid.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &CState.pidYaw.pid.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &CState.pidYaw.pid.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &CState.pidYaw.pid.kff)
/**
 * @brief If nonzero, yaw setpoint can only be set within +/- yawMaxDelta from the current yaw
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yawMaxDelta, &yawMaxDelta)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, attFiltEn, &attFiltEnable)
/**
 * @brief Target Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, attTFiltEn, &attTFiltEnable)
/**
 * @brief Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, attFiltCut, &attFiltCutoff)
/**
 * @brief Target Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, attTFiltCut, &attTFiltCutoff)
PARAM_GROUP_STOP(pid_attitude)

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates 
 */
PARAM_GROUP_START(pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &CState.pidRollRate.pid.kp)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &CState.pidRollRate.pid.ki)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &CState.pidRollRate.pid.kd)
/**
 * @brief Feedforward gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &CState.pidRollRate.pid.kff)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &CState.pidPitchRate.pid.kp)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &CState.pidPitchRate.pid.ki)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &CState.pidPitchRate.pid.kd)
/**
 * @brief Feedforward gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &CState.pidPitchRate.pid.kff)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &CState.pidYawRate.pid.kp)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &CState.pidYawRate.pid.ki)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &CState.pidYawRate.pid.kd)
/**
 * @brief Feedforward gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &CState.pidYawRate.pid.kff)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, rateFiltEn, &rateFiltEnable)
/**
 * @brief Target Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, rateTFiltEn, &rateTFiltEnable)
/**
 * @brief Low pass filter cut-off frequency, roll axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omxFiltCut, &omxFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, pitch axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omyFiltCut, &omyFiltCutoff)
/**
 * @brief Low pass filter cut-off frequency, yaw axis (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, omzFiltCut, &omzFiltCutoff)
/**
 * @brief Target Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rateTFiltCut, &rateTFiltCutoff)
PARAM_GROUP_STOP(pid_rate)
