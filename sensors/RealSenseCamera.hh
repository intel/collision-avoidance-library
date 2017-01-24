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

#include <librealsense/rs.hpp>
#include <memory>
#include <mutex>
#include <vector>

#include <common/DepthCamera.hh>

class RealSenseCamera: public DepthCamera
{
  public:
    RealSenseCamera(size_t width, size_t height, unsigned int fps);

    std::vector<uint16_t> &get_depth_buffer() override;
    double get_horizontal_fov() override;
    double get_vertical_fov() override;

  private:
    std::mutex depth_buffer_mtx;
    std::vector<uint16_t> depth_buffer;

    std::shared_ptr<rs::context> ctx;
    rs::device *dev;

    double hfov;
    double vfov;
};

