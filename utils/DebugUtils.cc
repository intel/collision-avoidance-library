/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "DebugUtils.hh"
#include <stdio.h>

void rainbow_scale(double value, uint8_t rgb[])
{
    rgb[0] = rgb[1] = rgb[2] = 0;

    if (value <= 0.0)
        return;

    if (value < 0.25) { // RED to YELLOW
        rgb[0] = 255;
        rgb[1] = (uint8_t)255 * (value / 0.25);
    } else if (value < 0.5) { // YELLOW to GREEN
        rgb[0] = (uint8_t)255 * (1 - ((value - 0.25) / 0.25));
        rgb[1] = 255;
    } else if (value < 0.75) { // GREEN to CYAN
        rgb[1] = 255;
        rgb[2] = (uint8_t)255 * (value - 0.5 / 0.25);
    } else if (value < 1.0) { // CYAN to BLUE
        rgb[1] = (uint8_t)255 * (1 - ((value - 0.75) / 0.25));
        rgb[2] = 255;
    } else { // BLUE
        rgb[2] = 255;
    }
}
