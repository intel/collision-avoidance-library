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
 * @brief Mavlink quadcopter
 */

#include <memory>
#include <thread>
#include <cstdint>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "common/math.hh"
#include "common/common.hh"
#include "mavlink_vehicles.hh"

/**
 * @brief Interface to a Mavlink quadcopter
 */
class MavQuadCopter : public QuadCopter
{
    /**
     * @brief Local Port constructor.
     * @param local_port Set the port to which this MavQuadCopter instance will
     *                   be bounded to.
     */
    public: MavQuadCopter(uint16_t local_port);

    /**
     * @brief Local Port and Auto Control constructor.
     * @param local_port Set the port to which this MavQuadCopter instance will
     *                   be bounded to.
     * @param autorotate If set to true, the MavQuadCopter will rotate
     *                   automatically to point to the mission target at all
     *                   times.
     * @param autotakeoff If set to true, the vehicle will takeoff
     *                    automatically if it's landed.
     */
    public: MavQuadCopter(uint16_t local_port, bool autorotate, bool autotakeoff);

    /**
     * @brief Default Constructor.
     */
    public: MavQuadCopter();

    /**
     * @brief Default Destructor.
     */
    public: ~MavQuadCopter();

    /**
     * @brief Implementation of target_pose getter.
     * @return The pose of the current mission target.
     */
    public: Pose target_pose() override;

    /**
     * @brief Implementation of vehicle_pose getter.
     * @return The current pose of the vehicle.
     */
    public: Pose vehicle_pose() override;

    /**
     * @brief Implementation of the mission target pose setter.
     * @param pose New mission target pose
     */
    public: void set_target_pose(Pose pose) override;

    /**
     * @brief Command the vehicle to immediately stop and rotate before moving
     *        to the next detour or mission waypoint.
     * @param angle_rad Rotation angle in radians (Right handed, Z-up)
     */
    public: void rotate(double angle_deg);

    /**
     * @brief Check if the vehicle is in detour mode.
     * @return True if not in detour mode. False otherwise.
     */
    public: bool detour_finished();

    /**
     * @brief Provides direct access to the complete interface of a mavlink
     * vehicle.
     */
    public: std::shared_ptr<mavlink_vehicles::mav_vehicle> mav;

  private:

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
};

