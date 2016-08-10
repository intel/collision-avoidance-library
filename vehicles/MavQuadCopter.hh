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

#include <memory>
#include <thread>
#include <cstdint>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "common/math.hh"
#include "common/common.hh"
#include "modules/mavlink_vehicles/mavlink_vehicles/mavlink_vehicles.hh"

class MavQuadCopter : public QuadCopter
{
  public:
    MavQuadCopter(uint16_t local_port);
    MavQuadCopter(uint16_t local_port, bool autorotate, bool autotakeoff);
    MavQuadCopter();
    ~MavQuadCopter();

    Pose target_pose() override;
    Pose vehicle_pose() override;
    void set_target_pose(Pose pose) override;
    void rotate(double angle_deg);

  private:

    // Mavlink vehicle
    std::shared_ptr<mavlink_vehicles::mav_vehicle> mav;

    // Connection
    int sock = 0;
    socklen_t fromlen = {0};
    struct sockaddr_in local_addr = {0};
    struct sockaddr_in remote_addr = {0};

    // Run Thread
    void run();
    std::thread thread;
    bool thread_run = false;

    // Vehicle state
    enum vstate { INIT, INIT_ON_GROUND, ACTIVE_ON_GROUND, ACTIVE_AIRBORNE };
    vstate vehicle_state = INIT;

    bool autotakeoff = false;
    bool autorotate = false;

    mavlink_vehicles::global_pos_int curr_detour_wp;
    mavlink_vehicles::global_pos_int prev_detour_wp;
};

