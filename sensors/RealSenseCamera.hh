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
 * @brief Realsense depth camera
 */

#include <librealsense/rs.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include <common/DepthCamera.hh>

/**
 * @brief Interface to a physical RealSense depth camera
 */
class RealSenseCamera: public DepthCamera
{
    /**
     * @brief Default Constructor.
     */
    public: RealSenseCamera(size_t width, size_t height, unsigned int fps);

    /**
     * @brief Get the most recent depth buffer.
     * @return Depth buffer in uint16_t. To convert to meters multiply these
     *         values by depth camera scale.
     */
    public: std::vector<uint16_t> &get_depth_buffer() override;

  private:
    std::mutex depth_buffer_mtx;
    std::vector<uint16_t> depth_buffer;

    std::shared_ptr<rs::context> ctx;
    rs::device *dev;
};

