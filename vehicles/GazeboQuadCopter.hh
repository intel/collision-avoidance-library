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
 * @brief Gazebo quadcopter
 */

#include <gazebo/transport/transport.hh>
#include <communication/GazeboContext.hh>
#include "common/common.hh"
#include <mutex>

/**
 * @brief Interface to a Gazebo virtual quadcopter
 */
class GazeboQuadCopter : public QuadCopter
{
  public:
    GazeboQuadCopter();
    ~GazeboQuadCopter();

    Pose target_pose() override;
    Pose vehicle_pose() override;
    void set_target_pose(Pose pose) override;

  private:
    void on_target_pose_recvd(ConstPosePtr &_msg);
    void on_vehicle_pose_recvd(ConstPosePtr &_msg);
    void on_stream_depth_recvd(ConstImageStampedPtr &_msg);

    std::shared_ptr<GazeboContext> gazebo_context;
    gazebo::transport::NodePtr gznode;
    gazebo::transport::SubscriberPtr target_pose_sub;
    gazebo::transport::SubscriberPtr vehicle_pose_sub;
    gazebo::transport::PublisherPtr coav_target_pose_pub;

    std::mutex target_pose_mtx;
    Pose target_pose_;
    std::mutex vehicle_pose_mtx;
    Pose vehicle_pose_;
};

