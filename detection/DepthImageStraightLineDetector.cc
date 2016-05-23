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

#include "DepthImageStraightLineDetector.hh"

#include <algorithm>
#include <iostream>
#include <glm/glm.hpp>

static const int DS_FACTOR = 64; // Downsampling factor
static const int DS_FACTOR_2 = DS_FACTOR * DS_FACTOR; // Downsampling factor squared

DepthImageStraightLineDetector::DepthImageStraightLineDetector(std::shared_ptr<DepthCamera> _depth_camera)
    : depth_camera(_depth_camera)
{
}

DepthImageStraightLineDetector::~DepthImageStraightLineDetector()
{
}

const std::vector<Obstacle> &DepthImageStraightLineDetector::detect()
{
    auto buffer = this->depth_camera->get_depth_buffer();
    // this->obstacles.resize(buffer.size() / (DS_FACTOR * DS_FACTOR));
    this->obstacles.resize(buffer.size() / DS_FACTOR_2);
    auto width = this->depth_camera->get_width();
    auto height = this->depth_camera->get_height();

    // Cleanup obstacles
    for (auto& obstacle : this->obstacles)
        obstacle.center.z = -10; // Using z as depth

    // Getting min 
    if (buffer.size() != width * height || obstacles.size() == 0)
        return this->obstacles;

    for (uint i = 0; i < height; i++) {
        for (uint j = 0; j < width; j++) {
            int index = i * width + j;
            int ds_width = width / DS_FACTOR;
            int ds_index = std::min(((i / DS_FACTOR) * ds_width) + (j / DS_FACTOR), uint(this->obstacles.size() - 1));
            double depth = buffer[index] * this->depth_camera->get_scale();

            if (this->obstacles[ds_index].center.z > depth && this->obstacles[ds_index].center.z > 0)
                continue;

            this->obstacles[ds_index].center = depth * glm::dvec3(
                j - (float(width) / 2),
                i - (float(height) / 2),
                1
            );
        }
    }

    return this->obstacles;
}

