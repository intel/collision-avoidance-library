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
#include <common/common.hh>
#include <memory>
#include <vector>
#include <cmath>

#include "DepthImageSimpleDetector.hh"

namespace defaults
{
const unsigned int radius_px = 30;
const double min_dist_m = 5.0;
}

DepthImageSimpleDetector::DepthImageSimpleDetector(
    std::shared_ptr<DepthCamera> depth_camera)
{
    this->sensor = depth_camera;
    this->threshold = defaults::min_dist_m;
}

DepthImageSimpleDetector::DepthImageSimpleDetector(
    std::shared_ptr<DepthCamera> depth_camera, double threshold_m)
{
    this->sensor = depth_camera;
    this->threshold = threshold_m;
}

const std::vector<bool> &DepthImageSimpleDetector::detect()
{
    // Obtain camera depth buffer and camera properties
    std::vector<uint16_t> depth_buffer = this->sensor->get_depth_buffer();
    unsigned int height = this->sensor->get_height();
    unsigned int width = this->sensor->get_width();
    double scale = this->sensor->get_scale();

    // Return false if depth buffer is empty
    if(depth_buffer.size() == 0) {
        this->detection[0] = false;
        return this->detection;
    }

    // Detect obstacles in the center of the image
    int init_i = fmin(0, height / 2 - defaults::radius_px);
    int final_i = fmin(height / 2 + defaults::radius_px, height);
    int init_j = fmin(0, width / 2 - defaults::radius_px);
    int final_j = fmin(width / 2 + defaults::radius_px, width);

    // Detect obstacles in the center of the image
    this->detection[0] = false;
    for (int i = init_i; i < final_i; i++) {
        for (int j = init_j; j < final_j; j++) {
            uint16_t depth_value = depth_buffer[i * width + j];
            if (depth_value != 0 &&
                depth_value < this->threshold / scale) {
                this->detection[0] = true;
                break;
            }
        }
    }

    return this->detection;
}

