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

#include <algorithm>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <thread>

#include "avoidance/QuadCopterShiftAvoidance.hh"
#include "vehicles/MavQuadCopter.hh"
#include "detection/DepthImagePolarHistDetector.hh"
#include "common/common.hh"
#include "common/DepthCamera.hh"

#ifdef HAVE_GAZEBO
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "sensors/GazeboRealSenseCamera.hh"
#include "vehicles/GazeboQuadCopter.hh"
#endif

#ifdef HAVE_REALSENSE
#include "sensors/RealSenseCamera.hh"
#endif

enum device_type_ { GAZEBO, PHYSICAL, OTHER };

static device_type_ vehicle_type = OTHER;
static device_type_ depth_camera_type = OTHER;

static void print_usage();
static device_type_ parse_device_type(const char *optarg);


// Definitions
bool running = true;

static void print_usage()
{
    std::cout << "Usage: coav [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --depth-camera GAZEBO|PHYSICAL    Depth Camera Type"
              << std::endl;
    std::cout << "  --vehicle GAZEBO|PHYSICAL         Vehicle Type"
              << std::endl;
    std::cout << "  --help                            Display Help"
              << std::endl;
}

static device_type_ parse_device_type(const char *optarg)
{
    device_type_ ret_val = OTHER;

    if (optarg) {
        std::string optarg_str(optarg);
        std::transform(optarg_str.begin(), optarg_str.end(), optarg_str.begin(),
                       ::tolower);

        if (optarg_str == std::string("gazebo")) {
            ret_val = GAZEBO;
        }

        if (optarg_str == std::string("physical")) {
            ret_val = PHYSICAL;
        }
    }

    return ret_val;
}

int main(int argc, char **argv)
{
    // Parse command line arguments
    bool args_need_help = false;
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"depth-camera", required_argument, 0, 'c'},
        {"vehicle", required_argument, 0, 'v'},
        {0, 0, 0, 0}};

    while (true) {
        int option_index = 0;
        int c = getopt_long(argc, argv, "hc:v:d", long_options, &option_index);

        if (c == -1) {
            break;
        }

        switch (c) {
        case 'c':
            depth_camera_type = parse_device_type(optarg);
            if (depth_camera_type == OTHER) {
                std::cout << "err: invalid depth camera type." << std::endl;
                args_need_help = true;
            }
            break;
        case 'v':
            vehicle_type = parse_device_type(optarg);
            if (vehicle_type == OTHER) {
                std::cout << "err: invalid vehicle type." << std::endl;
                args_need_help = true;
            }
            break;
        case 'h':
        default:
            print_usage();
            return 0;
        }
    }

    if (args_need_help) {
        print_usage();
        return 0;
    }

    // Check if camera and vehicle types were given
    if (depth_camera_type == OTHER || vehicle_type == OTHER) {
        std::cout << "Please provide valid device types" << std::endl;
        return 0;
    }

    // TODO: Implement PHYSICAL device support
    if (vehicle_type == PHYSICAL) {
        std::cout << "Physical devices are not supported yet" << std::endl;
        return 0;
    }

#ifndef HAVE_REALSENSE
    if (depth_camera_type == PHYSICAL) {
        std::cout << "Physical camera are not supported" << std::endl;
        return 0;
    }
#endif

    std::shared_ptr<DepthCamera> depth_camera;
    std::shared_ptr<MavQuadCopter> vehicle;
    std::shared_ptr<DepthImagePolarHistDetector> obstacle_detector;

    // Initialize Sensors
    switch (depth_camera_type) {
    case PHYSICAL:
#ifdef HAVE_REALSENSE
        depth_camera = std::make_shared<RealSenseCamera>(640, 480, 30);
        std::cout << "[coav] RealSenseCamera instantiated" << std::endl;
        break;
#else
        std::cout << "Physical camera requested but support wasn't found." << std::endl;
        return 0;
#endif
    case GAZEBO:
#ifdef HAVE_GAZEBO
        depth_camera = std::make_shared<GazeboRealSenseCamera>();
        std::cout << "[coav] GazeboRealSenseCamera instantiated" << std::endl;
        break;
#else
        std::cout << "Gazebo simulated camera requested but support wasn't found." << std::endl;
        return 0;
#endif
    case OTHER:
    default:
        std::cout << "At least one valid Depth Camera is needed to run this example but none was found." << std::endl;
        return 0;
    }

    obstacle_detector = std::make_shared<DepthImagePolarHistDetector>(
            depth_camera, 180.0 * (atan(depth_camera->get_fov_tan())) / M_PI);
    std::cout << "[coav] ObstacleDetector instantiated" << std::endl;

    // Initialize Vehicle
    vehicle = std::make_shared<MavQuadCopter>();
    std::cout << "[coav] Vehicle instantiated" << std::endl;

    // Initialize Avoidance Strategy
    auto avoidance = std::make_shared<QuadCopterShiftAvoidance>(vehicle);

    while (running) {
        // Sense
        std::vector<Obstacle> obstacles = obstacle_detector->detect();

        // Avoid
        avoidance->avoid(obstacles);
    }
}
