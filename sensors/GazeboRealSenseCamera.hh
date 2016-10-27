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

#include <gazebo/transport/transport.hh>
#include <communication/GazeboContext.hh>
#include <common/common.hh>
#include <mutex>
#include <vector>
#include <memory>

class GazeboRealSenseCamera: public DepthCamera
{
  public:
    GazeboRealSenseCamera();
    ~GazeboRealSenseCamera();

    std::vector<uint16_t> &get_depth_buffer();
    unsigned int get_height();
    unsigned int get_width();
    double get_scale();
    double get_fov_tan();

  private:
    void on_stream_depth_recvd(ConstImageStampedPtr &_msg);

    std::mutex depth_buffer_mtx;
    std::vector<uint16_t> depth_buffer[2] = {{}, {}};
    uint current_buffer = 0;
    std::shared_ptr<GazeboContext> gazebo_context;
    gazebo::transport::NodePtr gznode;
    gazebo::transport::SubscriberPtr rs_depth_sub;
    bool camera_exists = false;

    unsigned int width = 0;
    unsigned int height = 0;
    double fov = 0;
    double scale = 0;
};

