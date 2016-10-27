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
#include "QuadCopterShiftAvoidance.hh"
#include "vehicles/MavQuadCopter.hh"
#include "common/common.hh"
#include "common/math.hh"
#include "glm/glm.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <limits>
#include <iostream>

namespace defaults
{
const double safe_distance = 8.0;
const double lowest_altitude = 1.0;
const double detour_wp_angle = M_PI/3.0;
}

QuadCopterShiftAvoidance::QuadCopterShiftAvoidance(
    std::shared_ptr<MavQuadCopter> quadcopter)
{
    this->vehicle = quadcopter;
}

void QuadCopterShiftAvoidance::avoid(const std::vector<double> &histogram)
{
    Pose vehicle_pose = vehicle->vehicle_pose();

    if (!vehicle->mav->is_ready()) {
        return;
    }

    vehicle->mav->set_autorotate_during_mission(true);
    vehicle->mav->set_autorotate_during_detour(false);

    switch (this->avoidance_state) {
    case avoid_state::detouring: {
        // The vehicle is detouring. We need to wait for it to finish the
        // detour before checking for obstacles again.

        if(vehicle->detour_finished()) {
            this->avoidance_state = avoid_state::moving;
            std::cout << "[avoid] state = moving..." << std::endl;
        }

        break;
    }
    case avoid_state::moving: {
        // The vehicle is moving to the target. We need to detect if an
        // obstacle is found in order to start a detour.

        // The histogram does not have valid data. We're blind.
        if (histogram.size() == 0) {
            break;
        }

        // Check if the path in front of the vehicle is safe
        if (histogram[0] == 0 || histogram[0] > defaults::safe_distance) {
            break;
        }

        // Check if the current mission waypoint is closer than the closest obstacle
        if (mavlink_vehicles::math::ground_dist(
                vehicle->mav->get_global_position_int(),
                vehicle->mav->get_mission_waypoint()) <= histogram[0]) {
            break;
        }

        // Check if we are too close to ground
        if (vehicle_pose.pos.z < defaults::lowest_altitude) {
            break;
        }

        // Calculate the lookat vector
        glm::dvec3 view_dir =
            glm::dvec3(-sin(vehicle_pose.yaw()), cos(vehicle_pose.yaw()), 0);

        // Calculate the direction of the detour waypoint (horizontal rotation
        // around  lookat vector)
        glm::dvec3 wp_dir =
            glm::dvec3(view_dir.x * cos(defaults::detour_wp_angle) -
                           view_dir.y * sin(defaults::detour_wp_angle),
                       view_dir.x * sin(defaults::detour_wp_angle) +
                           view_dir.y * cos(defaults::detour_wp_angle),
                       0.0);

        // Calculate the global position of the waypoint
        Pose wp =
            Pose{vehicle_pose.pos + glm::abs(2.0) * glm::normalize(wp_dir),
                 glm::dquat(0, 0, 0, 0)};

        // Send the detour waypoint to the vehicle
        vehicle->set_target_pose(wp);

        // Update avoidance state
        this->avoidance_state = avoid_state::detouring;

        // Print info
        std::cout << "[avoid] state = detouring..." << std::endl;

        std::cout << "[avoid] wp (x, y, z): " << wp.pos.x << ", " << wp.pos.y
                  << "," << wp.pos.z << std::endl;

        std::cout << "[avoid] vh (x, y, z): " << vehicle_pose.pos.x << ", "
                  << vehicle_pose.pos.y << "," << vehicle_pose.pos.z
                  << std::endl;

        break;
    }
    }

    return;
}

