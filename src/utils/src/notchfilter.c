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

void NotchInit(NotchData* data, float sample_freq, float peak_freq, float bandwidth)
{
  data->peak  = peak_freq;
  data->bw    = bandwidth;
  data->a1    = 0.0f;
  data->a2    = 0.0f;
  data->b0    = 1.0f;
  data->b1    = 0.0f;
  data->b2    = 0.0f;
  data->delay_element_1 = 0.0f;
  data->delay_element_2 = 0.0f;
  data->delay_output_1  = 0.0f;
  data->delay_output_2  = 0.0f;
    
  if (data == NULL || peak_freq <= 0.0f || bandwidth <= 0.0f) {
    return;
  }

  NotchSetParameter(data, sample_freq);
}

void NotchSetParameter(NotchData* data, float sample_freq)
{
  float alpha   =  tanf(M_PI_F * data->bw / data->peak);
  float beta    = -cosf(2.0f * M_PI_F * data->peak / sample_freq);
  float a0_inv  =  1.0f / (alpha + 1.0f);

  data->b0 = a0_inv;
  data->b1 = 2.0f * beta * a0_inv;
  data->b2 = a0_inv;

  data->a1 = data->b1;
  data->a2 = (1.0f -alpha) * a0_inv;

  data->delay_element_1 = 0.0f;
  data->delay_element_2 = 0.0f;
  data->delay_output_1  = 0.0f;
  data->delay_output_2  = 0.0f;
}

float NotchApply(NotchData* data, float sample)
{
  // Direct form I implementation
  float ouput = data->b0 * sample + data->b1 * data->delay_element_1 + data->b2 * data->delay_element_2
                - data->a1 * data->delay_output_1 - data->a2 * data->delay_output_2;

  // Shift inputs
  data->delay_element_2 = data->delay_element_1;
  data->delay_element_1 = sample;

  // Shift outputs
  data->delay_output_2 = data->delay_output_1;
  data->delay_output_1 = ouput;
  
  return ouput;
}

float NotchReset(NotchData* data, float sample)
{
  float dval = sample * (data->b0 + data->b1 + data->b2)/(1.0f + data->a1 + data->a2);
  data->delay_element_1 = sample;
  data->delay_element_2 = sample;
  data->delay_output_2  = dval;
  data->delay_output_1  = dval;
  return NotchApply(data, sample);
}
