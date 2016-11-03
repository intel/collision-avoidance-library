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

#include <iostream>
#include <mutex>

#include "RealSenseCamera.hh"

#define DEPTH_CAM_WIDTH 640
#define DEPTH_CAM_HEIGHT 480
#define DEPTH_CAM_FOV M_PI / 3.0
#define DEPTH_CAM_SCALE 0.001
#define DEPTH_CAM_FPS 30

RealSenseCamera::RealSenseCamera() try {
    this->width = DEPTH_CAM_WIDTH;
    this->height = DEPTH_CAM_HEIGHT;
    this->fov = DEPTH_CAM_FOV;
    this->scale = DEPTH_CAM_SCALE;

    this->ctx = std::make_shared<rs::context>();
    if (this->ctx->get_device_count() == 0) {
        std::cerr << "[RealSenseCamera] No RealSense device found." << std::endl;
        return;
    }

    this->dev = this->ctx->get_device(0);
    this->dev->enable_stream(rs::stream::depth, this->width, this->height,
            rs::format::z16, DEPTH_CAM_FPS);
    this->dev->start();

    this->depth_buffer.reserve(this->width * this->height);
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
