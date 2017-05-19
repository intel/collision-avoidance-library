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

#include <cstdint>
#include <memory>
#include <vector>

struct DepthData
{
    unsigned int height;
    unsigned int width;
    double scale;
    double hfov;
    double vfov;
    std::vector<uint16_t> depth_buffer;
};

class DepthCamera
{
public:
    unsigned int get_height();
    unsigned int get_width();
    double get_scale();
    std::shared_ptr<struct DepthData> read();

    virtual std::vector<uint16_t> &get_depth_buffer() = 0;
    virtual double get_horizontal_fov() { return hfov; };
    virtual double get_vertical_fov() { return vfov; };

protected:
    unsigned int height = 0;
    unsigned int width = 0;
    double scale = 0;
    double hfov = 0.0;
    double vfov = 0.0;
};
