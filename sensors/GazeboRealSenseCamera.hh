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
 * @brief Gazebo realsense depth camera
 */

#include <gazebo/transport/transport.hh>
#include <common/DepthCamera.hh>
#include <communication/GazeboContext.hh>
#include <mutex>
#include <vector>
#include <memory>

/**
 * @brief Interface to a Gazebo virtual RealSense depth camera
 */
class GazeboRealSenseCamera: public DepthCamera
{
    /**
     * @brief Default Constructor.
     */
    public: GazeboRealSenseCamera();

    /**
     * @brief Default Destructor.
     */
    public: ~GazeboRealSenseCamera();

    /**
     * @brief Get the most recent depth buffer.
     * @return Depth buffer in uint16_t. To convert to meters multiply these
     *         values by depth camera scale.
     */
    public: std::vector<uint16_t> &get_depth_buffer() override;

  private:
    void on_stream_depth_recvd(ConstImageStampedPtr &_msg);

    std::mutex depth_buffer_mtx;
    std::vector<uint16_t> depth_buffer[2] = {{}, {}};
    uint current_buffer = 0;
    std::shared_ptr<GazeboContext> gazebo_context;
    gazebo::transport::NodePtr gznode;
    gazebo::transport::SubscriberPtr rs_depth_sub;
    bool camera_exists = false;
};

