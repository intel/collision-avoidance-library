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

#include <common/common.hh>
#include <common/DepthCamera.hh>
#include <vector>

class DepthImageObstacleDetector : public Detector<DepthCamera>
{

  public:
    DepthImageObstacleDetector(std::shared_ptr<DepthCamera> depth_camera, double threshold_meters = 0.0);
    const std::vector<Obstacle> &detect() override;

  private:
    std::vector<Obstacle> obstacles;
    std::vector<uint16_t> depth_frame;
    std::vector<uint16_t> labels;
    int width;
    int height;
    double hfov;
    double vfov;
    double scale;
    double base_theta;
    double base_phi;

    bool is_valid(const uint16_t depth);
    bool is_in_range(const uint16_t d1, const uint16_t d2);

    int get_neighbors_label(const int i, const int j, std::vector<int> &neigh_labels);

    int extract_blobs();

    double calc_pixel_area(int i, int j, uint16_t depth_value);

    int max_num_obstacles = 1000;
    int tolerance = 20;
    int min_num_pixels = 400; // equivalent area of a 20x20 square

    uint16_t threshold = 0;
};

