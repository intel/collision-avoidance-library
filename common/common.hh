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

#include <memory>
#include <vector>

#include "math.hh"

class QuadCopter
{
  public:
    virtual Pose target_pose() = 0;
    virtual Pose vehicle_pose() = 0;
    virtual void set_target_pose(Pose pose) = 0;
};

struct Obstacle {
    uint id;

    /**
     * Spherical coordinates as in ISO 31-11.
     *
     * x = r (Radial)
     * y = theta (Polar Angle)
     * z = phi (Azimuthal Angle)
     *
     * When returned from a Sensor, the origin is on it
     * and looking along the y axis.
     */
    glm::dvec3 center;
};

template <typename SensorType>
class Detector
{
  public:
    virtual const std::vector<Obstacle> &detect() = 0;

  protected:
    std::shared_ptr<SensorType> sensor;
};

template <typename VehicleType>
class CollisionAvoidanceStrategy
{
  public:
    virtual void avoid(const std::vector<Obstacle> &elements) = 0;

  protected:
    std::shared_ptr<VehicleType> vehicle;
};

