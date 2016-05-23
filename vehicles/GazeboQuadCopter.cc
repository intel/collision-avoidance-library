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

#include "GazeboQuadCopter.hh"

// TODO: Retrieve these values from the plugin
#define COAV_TARGET_PUB_PERIOD_MS 100
#define COAV_TARGET_PUB_FREQ_HZ (1000 / COAV_TARGET_PUB_PERIOD_MS)

#define GZ_QUAD_COPTER_TARGET_POSE_TOPIC "~/gzsitl_quadcopter_rs/target_pose"
#define GZ_QUAD_COPTER_POSE_TOPIC "~/gzsitl_quadcopter_rs/vehicle_pose"
#define GZ_COAV_TARGET_POSE_TOPIC "~/coav/coav_target_pose"

GazeboQuadCopter::GazeboQuadCopter()
{

    // Start communication with Gazebo
    std::cout << "[GazeboQuadCopter] Waiting for Gazebo..." << std::endl;
    this->gazebo_context = GazeboContext::instance();

    // Create our node for communication
    this->gznode.reset(gazebo_context->node());
    this->gznode->Init();

    std::cout << "[GazeboQuadCopter] Initialized" << std::endl;

    // TODO: Find GazeboQuadCopter camera topics and parameters automatically

    // Listen to Gazebo target waypoint topic
    this->target_pose_sub = this->gznode->Subscribe(
        GZ_QUAD_COPTER_TARGET_POSE_TOPIC, &GazeboQuadCopter::on_target_pose_recvd, this);

    // Listen to Gazebo GazeboQuadCopter pose topic
    this->vehicle_pose_sub = this->gznode->Subscribe(
        GZ_QUAD_COPTER_POSE_TOPIC, &GazeboQuadCopter::on_vehicle_pose_recvd, this);

    // Setup Coav Waypoint Publisher
    this->coav_target_pose_pub = this->gznode->Advertise<gazebo::msgs::Pose>(
        GZ_COAV_TARGET_POSE_TOPIC, 1, COAV_TARGET_PUB_FREQ_HZ);
}

GazeboQuadCopter::~GazeboQuadCopter()
{
}

static Pose pose_from_msg(ConstPosePtr &_msg)
{
    const auto pos = _msg->position();
    const auto rot = _msg->orientation();
    Pose pose;
    pose.pos.x = pos.x();
    pose.pos.y = pos.y();
    pose.pos.z = pos.z();
    pose.rot.x = rot.x();
    pose.rot.y = rot.y();
    pose.rot.z = rot.z();
    pose.rot.w = rot.w();
    return pose;
}

Pose GazeboQuadCopter::target_pose()
{
    std::lock_guard<std::mutex> locker(target_pose_mtx);
    return target_pose_;
}

Pose GazeboQuadCopter::vehicle_pose()
{
    std::lock_guard<std::mutex> locker(vehicle_pose_mtx);
    return vehicle_pose_;
}

void GazeboQuadCopter::set_target_pose(Pose pose)
{
    gazebo::math::Vector3 pos(pose.pos.x, pose.pos.y, pose.pos.z);
    gazebo::math::Quaternion rot(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
    gazebo::math::Pose gazebo_pose(pos, rot);
    this->coav_target_pose_pub->Publish(gazebo::msgs::Convert(gazebo_pose.Ign()));
}

void GazeboQuadCopter::on_target_pose_recvd(ConstPosePtr &_msg)
{
    if (!_msg->has_position())
        return;
    if (!_msg->has_orientation())
        return;

    std::lock_guard<std::mutex> locker(vehicle_pose_mtx);
    this->target_pose_ = pose_from_msg(_msg);
}

void GazeboQuadCopter::on_vehicle_pose_recvd(ConstPosePtr &_msg)
{
    if (!_msg->has_position())
        return;
    if (!_msg->has_orientation())
        return;

    std::lock_guard<std::mutex> locker(vehicle_pose_mtx);
    this->vehicle_pose_ = pose_from_msg(_msg);
}
