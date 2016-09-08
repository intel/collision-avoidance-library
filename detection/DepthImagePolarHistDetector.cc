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
#include <glm/glm.hpp>
#include "DepthImagePolarHistDetector.hh"

namespace defaults
{
    const unsigned int vertical_sweep_pixels = 10;
    const double angle_step = 5.0;
}

DepthImagePolarHistDetector::DepthImagePolarHistDetector(
    std::shared_ptr<DepthCamera> depth_camera, double angle_step)
    : depth_camera(depth_camera), angle_step(angle_step)
{
}

DepthImagePolarHistDetector::~DepthImagePolarHistDetector()
{
}

std::vector<double> DepthImagePolarHistDetector::detect()
{

    // Obtain camera depth buffer and camera properties
    std::vector<uint16_t> depth_buffer = this->depth_camera->get_depth_buffer();
    unsigned int height = this->depth_camera->get_height();
    unsigned int width = this->depth_camera->get_width();
    double fov_tan = this->depth_camera->get_fov_tan();
    double scale = this->depth_camera->get_scale();

    // Return an empty histogram if the depth buffer is invalid
    if(depth_buffer.size() == 0) {
        this->histogram.resize(0);
        return histogram;
    }

    unsigned int histogram_size =
        glm::ceil(glm::atan(fov_tan) / glm::radians(this->angle_step));
    unsigned int middle_row = height / 2;

    // Make sure the chosen vertical sweep area is within the limits of the
    // depth buffer
    unsigned int vertical_sweep_pixels =
        glm::min(defaults::vertical_sweep_pixels, middle_row);

    // Sweep a slice of the depth buffer filling up the histogram with the
    // closest distance found in a given direction
    this->histogram.resize(0);
    for (unsigned int i = (middle_row - vertical_sweep_pixels);
         i < (middle_row + vertical_sweep_pixels); i++) {
        for (unsigned int j = 0; j < width; j++) {
            unsigned int hist_pos = j / (width / histogram_size);
            uint16_t depth_value = depth_buffer[i * width + j];
            if (depth_value == 0) {
                depth_value = UINT16_MAX;
            }
            if (histogram.size() <= hist_pos) {
                histogram.push_back(depth_value * scale);
            } else {
                this->histogram[j / (width / histogram_size)] =
                    glm::min(depth_value * scale,
                             this->histogram[j / (width / histogram_size)]);
            }
        }
    }

    return this->histogram;
}

