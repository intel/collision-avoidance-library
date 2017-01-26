/*
// Copyright (c) 2017 Intel Corporation
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

#include <iostream>
#include <memory>
#include <vector>

#include "common/common.hh"
#include "common/DepthCamera.hh"
#include "avoidance/QuadCopterShiftAvoidance.hh"
#include "avoidance/QuadCopterStopAvoidance.hh"
#include "avoidance/QuadCopterVFFAvoidance.hh"
#include "detection/DepthImageObstacleDetector.hh"
#include "detection/DepthImagePolarHistDetector.hh"
#include "detection/DepthImageSimpleDetector.hh"
#include "detection/DepthImageStraightLineDetector.hh"

#include "vehicles/MavQuadCopter.hh"

#ifdef HAVE_REALSENSE
#include "sensors/RealSenseCamera.hh"
#endif

#ifdef HAVE_GAZEBO
#include "sensors/GazeboRealSenseCamera.hh"
#endif

#include "coav-control.hh"

using namespace std;

int main (int argc, char* argv[])
{
    control_options opts = parse_cmdline(argc, argv);

    if (!opts.quiet) {
        cout << "Using Detect Algorith: " << detect_to_name(opts.detect) << endl;
        cout << "Using Avoidance Algorith: " << avoidance_to_name(opts.avoidance) << endl;
        cout << "Using Sensor: " << sensor_to_name(opts.sensor) << endl;
    }

    shared_ptr<MavQuadCopter> vehicle = std::make_shared<MavQuadCopter>();

    shared_ptr<DepthCamera> sensor;

    if (opts.sensor == ST_REALSENSE) {
#ifdef HAVE_REALSENSE
        sensor = make_shared<RealSenseCamera>(640, 480, 30);
#endif
    } else if (opts.sensor == ST_GAZEBO_REALSENSE) {
#ifdef HAVE_GAZEBO
        sensor = make_shared<GazeboRealSenseCamera>();
#endif
    }

    if (opts.sensor == ST_UNDEFINED || sensor == nullptr) {
        cerr << "ERROR: Invalid Sensor" << endl;
        exit(-EINVAL);
    }

    shared_ptr<Detector<DepthCamera>> detector;
    switch (opts.detect) {
        case DI_OBSTACLE:
            detector = make_shared<DepthImageObstacleDetector>(sensor, 5.0);
            break;
        case DI_STRAIGHT_LINE:
            detector = make_shared<DepthImageStraightLineDetector>(sensor);
            break;
        case DI_POLAR_HIST:
            detector = make_shared<DepthImagePolarHistDetector>(sensor, 5);
            break;
        case DI_SIMPLE:
            detector = make_shared<DepthImageSimpleDetector>(sensor);
            break;
        default:
            cerr << "ERROR: Invalid Detector" << endl;
            exit(-EINVAL);
    }

    shared_ptr<CollisionAvoidanceStrategy<MavQuadCopter>> avoidance;
    switch(opts.avoidance) {
        case QC_SHIFT_AVOIDANCE:
            avoidance = make_shared<QuadCopterShiftAvoidance>(vehicle);
            break;
        case QC_STOP:
            avoidance = make_shared<QuadCopterStopAvoidance>(vehicle);
            break;
        case QC_VFF:
            avoidance = make_shared<QuadCopterVFFAvoidance>(vehicle);
            break;
        default:
            cerr << "ERROR: Invalid Avoidance" << endl;
            exit(-EINVAL);
    }

    while (true) {
        avoidance->avoid(detector->detect());
    }

    return 0;
}
