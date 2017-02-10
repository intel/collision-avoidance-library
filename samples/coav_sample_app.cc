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
#include <cmath>
#include <memory>
#include <vector>

#include "avoidance/QuadCopterStopAvoidance.hh"
#include "common/common.hh"
#include "detection/DepthImageSimpleDetector.hh"
#include "sensors/GazeboRealSenseCamera.hh"
#include "vehicles/MavQuadCopter.hh"

int main(int argc, char **argv)
{
    // Initialize Depth Camera
    std::shared_ptr<DepthCamera> depth_camera =
        std::make_shared<GazeboRealSenseCamera>();

    // Initialize Vehicle
    std::shared_ptr<MavQuadCopter> vehicle = std::make_shared<MavQuadCopter>();

    // Initialize Detector
    std::shared_ptr<DepthImageSimpleDetector> obstacle_detector =
        std::make_shared<DepthImageSimpleDetector>(depth_camera);

    // Initialize Avoidance Strategy
    std::shared_ptr<QuadCopterStopAvoidance> avoidance =
        std::make_shared<QuadCopterStopAvoidance>(vehicle);

    while (true) {

        // Sense and avoid
        auto sensed_elements = obstacle_detector->detect();

        // Avoid
        avoidance->avoid(sensed_elements);
    }
}

