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
#ifndef NOTCHFILTER_H_
#define NOTCHFILTER_H_
#include <stdint.h>
#include "math.h"

typedef struct {
  float peak;
  float bw;
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
  float delay_element_1;
  float delay_element_2;
  float delay_output_1;
  float delay_output_2;
} NotchData;

void NotchInit(NotchData* data, float sample_freq, float peak_freq, float bandwidth);
void NotchSetParameter(NotchData* data, float sample_freq);
float NotchApply(NotchData* data, float sample);
float NotchReset(NotchData* data, float sample);

#endif // NOTCHFILTER_H_
