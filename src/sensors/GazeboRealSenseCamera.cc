/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo/gazebo_client.hh>

#include "sensors/GazeboRealSenseCamera.hh"

// TODO: The camera configuration should be aggregated on the gazebo .sdf model
// file or retrieved from the librealsense. This feature is not implemented
// yet, so those values are stored here until then.
#define GZ_RS_STREAM_DEPTH_TOPIC "~/gzsitl_quadcopter_rs/rs/stream/depth"
#define DEPTH_CAM_WIDTH 640
#define DEPTH_CAM_HEIGHT 480
#define DEPTH_CAM_FOV M_PI / 3.0
#define DEPTH_CAM_SCALE 0.001

// =============
// GazeboContext
// =============

class GazeboContext
{
  public:
    ~GazeboContext();
    gazebo::transport::Node *node();
    static std::shared_ptr<GazeboContext> instance();

  private:
    GazeboContext();
};

GazeboContext::GazeboContext()
{
    gazebo::client::setup();
    std::cout << "[GazeboContext] Gazebo client has been set up" << std::endl;
}

GazeboContext::~GazeboContext()
{
    gazebo::client::shutdown();
    std::cout << "[GazeboContext] Gazebo client has been shut down" << std::endl;
}

std::shared_ptr<GazeboContext> GazeboContext::instance() {
    static std::weak_ptr<GazeboContext> _instance;
    if (auto ptr = _instance.lock()) { // .lock() returns a shared_ptr and increments the refcount
        return ptr;
    }
    // Does not support std::make_shared<GazeboContext> because of
    // the Resource private constructor.
    auto ptr = std::shared_ptr<GazeboContext>(new GazeboContext());
    _instance = ptr;
    return ptr;
}

gazebo::transport::Node *GazeboContext::node()
{
    return new gazebo::transport::Node();
}

// =====================
// GazeboRealSenseCamera
// =====================

GazeboRealSenseCamera::GazeboRealSenseCamera()
{
    // Start communication with Gazebo
    std::cout << "[GazeboRealSenseCamera] Waiting for Gazebo..." << std::endl;
    this->gazebo_context = GazeboContext::instance();

    // Create our node for communication
    this->gznode.reset(gazebo_context->node());
    this->gznode->Init();

    std::cout << "[GazeboRealSenseCamera] Gazebo Initialized" << std::endl;

    // TODO: Retrieve camera data straight from topic or camera plugin
    this->width = DEPTH_CAM_WIDTH;
    this->height = DEPTH_CAM_HEIGHT;
    this->hfov = DEPTH_CAM_FOV;
    this->scale = DEPTH_CAM_SCALE;

    // TODO: Find RealSense camera topic and parameters automatically
    this->rs_depth_sub = this->gznode->Subscribe(
        GZ_RS_STREAM_DEPTH_TOPIC, &GazeboRealSenseCamera::on_stream_depth_recvd,
        this);
}

GazeboRealSenseCamera::~GazeboRealSenseCamera()
{
}

std::vector<uint16_t> &GazeboRealSenseCamera::get_depth_buffer()
{
    std::lock_guard<std::mutex> locker(depth_buffer_mtx);
    return depth_buffer[this->current_buffer ^= 1];
}

void GazeboRealSenseCamera::on_stream_depth_recvd(ConstImageStampedPtr &_msg)
{
    if (!this->camera_exists) {
        this->camera_exists = true;
        std::cout << "[GazeboRealSenseCamera] Real Sense Initialized" << std::endl;
    }

    std::lock_guard<std::mutex> locker(depth_buffer_mtx);

    uint16_t *data = (uint16_t *) _msg->image().data().c_str();
    uint buffer_size = _msg->image().width() * _msg->image().height();
    depth_buffer[this->current_buffer ^ 1] = std::vector<uint16_t>(data, data + buffer_size);
}
