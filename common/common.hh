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
 * @brief Common base classes
 */

#include <memory>
#include <vector>

#include "math.hh"

/**
 * @brief Quadcopter base class
 */
class QuadCopter
{
    /**
     * @brief Pure virtual function that returns the pose of the current
     *        mission target.
     * @return The pose of the current mission target.
     */
    public: virtual Pose target_pose() = 0;

    /**
     * @brief Pure virtual function that returns the current pose of the
     *        vehicle.
     * @return The current pose of the vehicle.
     */
    public: virtual Pose vehicle_pose() = 0;

    /**
     * @brief Pure virtual function that sets the pose of the current mission
     *        target.
     * @param pose New mission target pose.
     */
    public: virtual void set_target_pose(Pose pose) = 0;
};

/**
 * @brief Class that represents an obstacle element
 */
struct Obstacle {
    uint id;
    glm::dvec3 center;
};

/**
 * @brief Detector base class
 */
template <typename SensorType, typename DetectedElementType>
class Detector
{
    /**
     * @brief Pure virtual function that returns the detected elements
     *        according to DetectedElementType.
     */
    public: virtual const std::vector<DetectedElementType> &detect() = 0;

    protected: std::shared_ptr<SensorType> sensor;
};

/**
 * @brief Collision Avoidance Strategy base class
 */
template <typename VehicleType, typename DetectedElementType>
class CollisionAvoidanceStrategy
{
    /**
     * @brief Pure virtual function that controls the vehicle according to the
     *        detected obstacles given as input.
     * @param elements Vector of detected elements.
     */
    public: virtual void avoid(const std::vector<DetectedElementType> &elements) = 0;

    protected: std::shared_ptr<VehicleType> vehicle;
};

