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
#pragma once

/**
 * @file
 * @brief Virtual force field avoidance strategy for quadcopters
 */

#include "common/common.hh"

#include <chrono>
#include <glm/glm.hpp>

/**
 * @brief Virtual Force Field Avoidance strategy for quadcopters.
 *        The current mission waypoint behaves as a source of attractive force
 *        while the obstacles behave as sources of repulsive force based on
 *        their dimension and distance from the vehicle. The vehicle changes
 *        its heading according to the resultant force trying to avoid
 *        obstacles.
 */
class QuadCopterVFFAvoidance : public CollisionAvoidanceStrategy<QuadCopter, Obstacle>
{
public:
  QuadCopterVFFAvoidance(std::shared_ptr<QuadCopter> quadcopter);
  void avoid(const std::vector<Obstacle> &obstacles) override;

private:
  bool coav_enabled = true;
  std::chrono::time_point<std::chrono::system_clock> last_calc_time =
      std::chrono::system_clock::from_time_t(0);

  glm::dvec3 calculate_coav_wp(Pose pose, double turn_rate, double climb_rate,
                               double closest_obst_dist, double target_dist);
};

