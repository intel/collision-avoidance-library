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

#include "sensors/RealSenseCamera.hh"

#include <iostream>

#include <glm/glm.hpp>

RealSenseCamera::RealSenseCamera(size_t width, size_t height, unsigned int fps) try
{
    this->width = width;
    this->height = height;

    this->ctx = std::make_shared<rs::context>();
    if (this->ctx->get_device_count() == 0) {
        std::cerr << "[RealSenseCamera] No RealSense device found." << std::endl;
        return;
    }

    this->dev = this->ctx->get_device(0);
    this->dev->enable_stream(rs::stream::depth, this->width, this->height,
            rs::format::z16, fps);

    auto intrinsics = this->dev->get_stream_intrinsics(rs::stream::depth);
    this->hfov = glm::radians(intrinsics.hfov());
    this->vfov = glm::radians(intrinsics.vfov());

    this->scale = this->dev->get_depth_scale();

    this->dev->start();

    this->depth_buffer.resize(this->width * this->height);
}
catch(const rs::error &e)
{
    std::cerr << "[RealSenseCamera] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;
}

std::vector<uint16_t> &RealSenseCamera::get_depth_buffer() try
{
    if (this->dev == nullptr) {
        std::cerr << "[RealSenseCamera] Error: No device available." << std::endl;
        return this->depth_buffer;
    }

    this->dev->wait_for_frames();

    const uint16_t *frame = reinterpret_cast<const uint16_t *>
        (this->dev->get_frame_data(rs::stream::depth));

    std::lock_guard<std::mutex> locker(depth_buffer_mtx);
    for (size_t i = 0; i < this->width * this->height; i++)
        this->depth_buffer[i] = frame[i];

    return this->depth_buffer;
}
catch(const rs::error &e)
{
    std::cerr << "[RealSenseCamera] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;

    return this->depth_buffer;
}
