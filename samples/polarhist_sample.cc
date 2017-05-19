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

#include <cmath>
#include <memory>
#include <vector>
#include <iostream>

#include <coav/coav.hh>

#define STEP (5.0 * M_PI / 180)

int main(int argc, char **argv)
{
    std::shared_ptr<DepthCamera> depth_camera =
        std::make_shared<RealSenseCamera>(640, 480, 30);

    std::shared_ptr<DepthImagePolarHistDetector> obstacle_detector =
        std::make_shared<DepthImagePolarHistDetector>(STEP, 1.5, 0.2);

    double fov = depth_camera->get_horizontal_fov();
    unsigned int slices = ceil(fov / STEP);
    double fixed_step = fov / slices;

    std::cout << "fov = " << fov << std::endl;
    std::cout << "step = " << fixed_step << std::endl;

    while (true) {
        std::vector<Obstacle> obstacles = obstacle_detector->detect(depth_camera->read());

        std::vector<bool> visual_slices;
        visual_slices.resize(slices, false);

        for (Obstacle obs : obstacles) {
            double offset = (M_PI / 2) - (fov / 2);
            int pos = slices - (obs.center.z - offset) / fixed_step;
            visual_slices[pos] = true;
        }

        for (unsigned int i = 0; i < slices; i++)
            std::cout << (visual_slices[i]?"████":"____");

        std::cout << "\r";
        std::cout.flush();
    }

    return 0;
}
