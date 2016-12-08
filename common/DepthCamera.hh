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
 * @brief Depth Camera base class
 */

#include <cstdint>
#include <vector>

/**
 * @brief Depth Camera base class
 */
class DepthCamera
{
    protected:
        unsigned int width = 0;
        unsigned int height = 0;
        double fov = 0;
        double scale = 0;

    /**
     * @brief Pure virtual function that returns a depth buffer as an uint16_t
     *        array.
     * @return Depth buffer.
     */
    public: virtual std::vector<uint16_t> &get_depth_buffer() = 0;

    /**
     * @brief Pure virtual function that returns the height of the depth
     *        buffer in pixels.
     * @return Depth buffer height in pixels.
     */
    public: unsigned int get_height();

    /**
     * @brief Pure virtual function that returns the width of the depth
     *        buffer in pixels.
     * @return Depth buffer width in pixels.
     */
    public: unsigned int get_width();

    /**
     * @brief Pure virtual function that returns the scale in meters in which
     *        the depth data is stored in the depth buffer. To convert the
     *        depth data to meters simply multiply it by the value returned by this
     *        function.
     * @return Scale in meters.
     */
    public: double get_scale();

    /**
     * @brief Pure virtual function that returns the tangent of the diagonal
     *        field of view of the depth camera.
     * @return Tangent of the diagonal field of view.
     */
    public: double get_fov_tan();
};

