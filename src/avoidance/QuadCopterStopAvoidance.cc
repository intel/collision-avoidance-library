/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#include <iostream>

#include "QuadCopterStopAvoidance.hh"
#include "common/common.hh"
#include "common/math.hh"
#include "vehicles/MavQuadCopter.hh"

namespace defaults
{
const float lowest_altitude = 2.0;
}

QuadCopterStopAvoidance::QuadCopterStopAvoidance(
    std::shared_ptr<MavQuadCopter> quadcopter, double trigger_distance)
{
    this->vehicle = quadcopter;
    this->trigger_dst = trigger_distance;
}

void QuadCopterStopAvoidance::avoid(const std::vector<Obstacle> &detection)
{
    if(!this->vehicle) {
        std::cout << "[avoid] vehicle does not exist" << std::endl;
        return;
    }

    if(!this->vehicle->mav) {
        std::cout << "[avoid] vehicle mav does not exist" << std::endl;
        return;
    }

    if (!this->vehicle->mav->is_ready()) {
        return;
    }

    // Vehicle is too close to ground. Do nothing.
    if (vehicle->vehicle_pose().pos.z < defaults::lowest_altitude) {
        return;
    }

    // If no obstacle was detected, do nothing.
    if (detection.size() == 0) {
        return;
    }

    // Send the stop command to the vehicle if any obstacle
    // is closer than or at trigger distance.
    for (Obstacle o : detection) {
        if (o.center.x <= this->trigger_dst) {
            if (!this->vehicle->mav->is_brake_active()) {
                this->vehicle->mav->brake(false);
                std::cout << "[avoid] state = stopping..." << std::endl;
            }
        }
    }

    return;
}

