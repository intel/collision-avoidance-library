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

#include "GazeboContext.hh"
#include <thread>

GazeboContext::GazeboContext()
{
    gazebo::client::setup();
    std::cout << "[GazeboContext] Gazebo client has ben set up" << std::endl;
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

