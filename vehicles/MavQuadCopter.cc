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
#include <arpa/inet.h>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "MavQuadCopter.hh"
#include "common/common.hh"
#include "common/math.hh"
#include "modules/mavlink_vehicles/mavlink_vehicles/mavlink_vehicles.hh"

namespace defaults
{
const uint16_t local_port = 14557;
const double wp_equal_dist_m = 0.001;
}

MavQuadCopter::MavQuadCopter() : MavQuadCopter(defaults::local_port)
{
}

MavQuadCopter::MavQuadCopter(uint16_t local_port)
{
    // Socket Initialization
    this->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        perror("[MavQuadCopter] error opening socket");
        exit(EXIT_FAILURE);
    }

    this->local_addr.sin_family = AF_INET;
    this->local_addr.sin_addr.s_addr = INADDR_ANY;
    this->local_addr.sin_port = htons(local_port);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) ==
        -1) {
        perror("[MavQuadCopter] error binding to port");
        close(sock);
        exit(EXIT_FAILURE);
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        perror("[MavQuadCopter] error setting socket as nonblocking");
        close(this->sock);
        exit(EXIT_FAILURE);
    }

    // Instantiate mav_vehicle
    this->mav = std::make_shared<mavlink_vehicles::mav_vehicle>(sock);
    std::cout << "[MavQuadCopter] mavlink_vehicle instantiated" << std::endl;

    // Initialize mav_vehicle update thread
    this->thread_run = true;
    this->thread = std::thread(&MavQuadCopter::run, this);
    this->thread.detach();
}

MavQuadCopter::~MavQuadCopter()
{
    this->thread_run = false;
}

void MavQuadCopter::run()
{
    while (this->thread_run) {

        // Update vehicle state
        this->mav->update();

        // Check if vehicle is ready
        if (!this->mav->is_ready()) {
            return;
        }

        mavlink_vehicles::arm_status arm_stat = this->mav->get_arm_status();
        mavlink_vehicles::status status = this->mav->get_status();
        mavlink_vehicles::mode mode = this->mav->get_mode();

        switch (this->vehicle_state) {
        case INIT: {

            if (status == mavlink_vehicles::status::STANDBY) {
                this->vehicle_state = INIT_ON_GROUND;
            } else if (status == mavlink_vehicles::status::ACTIVE) {
                this->vehicle_state = ACTIVE_AIRBORNE;
            }

            break;
        }
        case INIT_ON_GROUND: {

            if (!this->autotakeoff) {
                this->vehicle_state = ACTIVE_ON_GROUND;
                break;
            }

            // Execute takeoff procedures
            if (mode != mavlink_vehicles::mode::GUIDED) {
                this->mav->set_mode(mavlink_vehicles::mode::GUIDED);
                break;
            }

            if (arm_stat != mavlink_vehicles::arm_status::ARMED) {
                this->mav->arm_throttle();
                break;
            }

            if (status != mavlink_vehicles::status::ACTIVE) {
                this->mav->takeoff();
                break;
            }

            // Takeoff succeeded
            this->vehicle_state = ACTIVE_AIRBORNE;
            break;
        }
        case ACTIVE_ON_GROUND: {

            if (status == mavlink_vehicles::status::ACTIVE) {
                this->vehicle_state = ACTIVE_AIRBORNE;
            }

            break;
        }
        case ACTIVE_AIRBORNE: {

            // Do not send detour waypoint if it hasn't changed
            if (mavlink_vehicles::math::dist(curr_detour_wp, prev_detour_wp) <
                defaults::wp_equal_dist_m) {
                break;
            }

            // Detour wp has changed
            prev_detour_wp = curr_detour_wp;

            // TODO: Rotate if autorotate is enabled
            // if (this->autorotate) {
            // }

            // Send detour wp
            this->mav->send_detour_waypoint(curr_detour_wp);

            break;
        }

        }
    }
}

Pose MavQuadCopter::target_pose()
{
    using namespace mavlink_vehicles;

    // Convert from global_pos_int to local_pos_enu and return
    local_pos target_pos_ned = mavlink_vehicles::math::global_to_local_ned(
        this->mav->get_mission_waypoint(), this->mav->get_home_position_int());

    Pose target_pose;
    target_pose.pos.x = target_pos_ned.y;
    target_pose.pos.y = target_pos_ned.x;
    target_pose.pos.z = -target_pos_ned.z;
    target_pose.rot.x = 0;
    target_pose.rot.y = 0;
    target_pose.rot.z = 0;
    target_pose.rot.w = 0;

    return target_pose;
}

Pose MavQuadCopter::vehicle_pose()
{
    using namespace mavlink_vehicles;

    // Convert from global_pos_int to local_pos_enu and return
    local_pos vehicle_pos_ned = this->mav->get_local_position_ned();

    Pose vehicle_pose;
    vehicle_pose.pos.x = vehicle_pos_ned.y;
    vehicle_pose.pos.y = vehicle_pos_ned.x;
    vehicle_pose.pos.z = -vehicle_pos_ned.z;
    vehicle_pose.rot.x = 0;
    vehicle_pose.rot.y = 0;
    vehicle_pose.rot.z = 0;
    vehicle_pose.rot.w = 0;

    return vehicle_pose;
}

void MavQuadCopter::set_target_pose(Pose pose)
{
    using namespace mavlink_vehicles;

    // Convert from local_pos_enu to global_pos_int
    global_pos_int target_pos = mavlink_vehicles::math::local_ned_to_global(
        local_pos(pose.pos.y, pose.pos.x, -pose.pos.z),
        this->mav->get_home_position_int());

    curr_detour_wp = target_pos;
}

void MavQuadCopter::rotate(double angle_deg)
{
    this->mav->rotate(angle_deg);
}
