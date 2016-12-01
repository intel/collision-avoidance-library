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
 * @brief Obstacle detector for depth inputs
 */

#include <common/common.hh>
#include <common/DepthCamera.hh>
#include <vector>

/**
 * @brief Obstacle Detector based on Depth data
 */
class DepthImageObstacleDetector : public Detector<DepthCamera, Obstacle>
{

  public:
    DepthImageObstacleDetector(std::shared_ptr<DepthCamera> depth_camera);
    const std::vector<Obstacle> &detect() override;

  private:
    std::vector<Obstacle> obstacles;
    std::vector<uint16_t> labels;
    std::vector<uint16_t> curr_depth_frame;
    std::vector<uint16_t> curr_labels;
    int curr_width;
    int curr_height;
    double curr_fov;
    double curr_scale;

    int get_neighbors_label(int i, int j, int *neigh_labels);

    int extract_blobs();

    double calc_pixel_area(int i, int j, uint16_t depth_value);

    int max_num_obstacles = 1000;
    int color_tolerance = 2000;
    int min_num_pixels = 20;
    uint16_t bg_color = 0;
};

