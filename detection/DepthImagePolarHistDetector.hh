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
 * @brief Polar histogram detector for depth inputs
 */

#include <common/common.hh>
#include <common/DepthCamera.hh>
#include <memory>
#include <vector>

/**
 * @brief Polar Histogram detector based on Depth data
 */
class DepthImagePolarHistDetector : public Detector<DepthCamera, double>
{
    /**
     * @brief Default Constructor.
     * @param depth_camera Smart pointer to the depth_camera that will be used
     *                     as an input sensor to the detector.
     * @param angle_step Horizontal angle span that will be contained in each
     *                   element of the resulting histogram. For example, if
     *                   angle_step >= camera_fov, the number of elements of
     *                   the resulting histogram will be one.
     */
    public: DepthImagePolarHistDetector(std::shared_ptr<DepthCamera> depth_camera,
                                double angle_step);

    /**
     * @brief Generate a vector of doubles based on the data from the
     *        depth camera. Each element of the output vector contains the
     *        closest distance in meters found in that angle.
     * @return Reference to the generated vector of distances.
     */
    public: const std::vector<double> &detect() override;

  private:
    double angle_step;
    std::vector<double> histogram;
};

