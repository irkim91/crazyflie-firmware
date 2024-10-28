/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * filter.h - Filtering functions
 */

#include <math.h>
#include <stdlib.h>

#include "notchfilter.h"
#include "physicalConstants.h"

/**
 * notch filter (band pass filter)
 */
void notchfilterInit(notchData* notchData, float sample_freq, float notch_freq, float bandwidth)
{
  if (notchData == NULL || notch_freq <= 0.0f || bandwidth <= 0.0f) {
    return;
  }

  notchSetParameters(notchData, sample_freq, notch_freq, bandwidth);
}

void notchSetParameters(notchData* notchData, float sample_freq, float notch_freq, float bandwidth)
{
  float alpha = tanf(M_PI_F * bandwidth / sample_freq);
  float beta = -cosf(2.0f * M_PI_F * notch_freq / sample_freq);
  float a0_inv = 1.0f / (alpha + 1.0f);

  notchData->b0 = a0_inv;
  notchData->b1 = 2.0f * beta * a0_inv;
  notchData->b2 = a0_inv;

  notchData->a1 = notchData->b1;
  notchData->a2 = (1.0f - alpha) * a0_inv;

  notchData->delay_element_1 = 0.0f;
  notchData->delay_element_2 = 0.0f;
  notchData->delay_element_output_1 = 0.0f;
  notchData->delay_element_output_2 = 0.0f;
}

float notchfilterApply(notchData* notchData, float sample)
{
  // Direct Form I implementation
  float output = notchData->b0 * sample +  notchData->b1 *  notchData->delay_element_1 + notchData->b2 * notchData->delay_element_2 
                  -  notchData->a1 *  notchData->delay_element_output_1 -  notchData->a2 * notchData->delay_element_output_2;

  // shift inputs
  notchData->delay_element_2 = notchData->delay_element_1;
  notchData->delay_element_1 = sample;

  // shift outputs
	notchData->delay_element_output_2 = notchData->delay_element_output_1;
	notchData->delay_element_output_1 = output;

  return output;
}

float notchfilterReset(notchData* notchData, float sample)
{
  float dval = sample * (notchData->b0 + notchData->b1 + notchData->b2)/(1.0f + notchData->a1 + notchData->a2);
  notchData->delay_element_1 = sample;
  notchData->delay_element_2 = sample;
  notchData->delay_element_output_1 = dval;
  notchData->delay_element_output_2 = dval;
  return notchfilterApply(notchData, sample);
}
