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

#include <cmath>

#include "DepthImageObstacleDetector.hh"
#include "utils/PPTree.hh"
#include "common/common.hh"

#define MAX_NUM_BLOBS 3000
#define BACKGROUND 0

DepthImageObstacleDetector::DepthImageObstacleDetector(
    std::shared_ptr<DepthCamera> depth_camera)
{
    this->sensor = depth_camera;
    this->obstacles.resize(MAX_NUM_BLOBS);
}

const std::vector<Obstacle> &DepthImageObstacleDetector::detect()
{
    // Store current camera frame data
    this->depth_frame = this->sensor->get_depth_buffer();
    this->height = this->sensor->get_height();
    this->width = this->sensor->get_width();
    this->scale = this->sensor->get_scale();
    this->fov = this->sensor->get_fov_tan();

    // Detect obstacles from current depth buffer
    int num_obstacles = this->extract_blobs();

    // Return the obstacles with the correct size
    this->obstacles.resize(num_obstacles);

    return this->obstacles;
}

int DepthImageObstacleDetector::get_neighbors_label(int i, int j, int *neigh_labels)
{
    int pixel_idx, north_idx;
    int neighbor_idx = 0;

    if (i < 0 || i >= this->height || j < 0 || j >= this->width) {
        return 0;
    }

    pixel_idx = i * this->width + j;
    north_idx = pixel_idx - this->width;

    /* Check west */
    if (j > 0) {
        if (abs(depth_frame[pixel_idx - 1] -
                depth_frame[pixel_idx]) <= tolerance) {
            neigh_labels[neighbor_idx] = this->labels[pixel_idx - 1];
            neighbor_idx++;
        }
    }

    /* Check northwest */
    if (j > 0 && i > 0) {
        if (abs(depth_frame[north_idx - 1] -
                depth_frame[pixel_idx]) <= tolerance) {
            neigh_labels[neighbor_idx] = this->labels[north_idx - 1];
            neighbor_idx++;
        }
    }

    /* Check north */
    if (i > 0) {
        if (abs(depth_frame[north_idx] -
                depth_frame[pixel_idx]) <= tolerance) {
            neigh_labels[neighbor_idx] = this->labels[north_idx];
            neighbor_idx++;
        }
    }

    /* Check northeast */
    if (i > 0 && j < (this->width - 1)) {
        if (abs(depth_frame[north_idx + 1] -
                 depth_frame[pixel_idx]) <= tolerance) {
            neigh_labels[neighbor_idx] = this->labels[north_idx + 1];
            neighbor_idx++;
        }
    }

    /* Find lowest id and put it in front */
    int lowest_id = neigh_labels[0];
    for (int k = 1; k < neighbor_idx; k++) {
        if (neigh_labels[k] < lowest_id) {
            lowest_id = neigh_labels[k];
            neigh_labels[k] = neigh_labels[0];
            neigh_labels[0] = lowest_id;
        }
    }

    return neighbor_idx;
}

void init_obstacle_array(std::vector<Obstacle> &obstacles)
{
    obstacles.resize(MAX_NUM_BLOBS);
    for (unsigned int i = 0; i < obstacles.size(); ++i) {
        obstacles[i].id = -1;
        obstacles[i].center = glm::dvec3(-1, -1, -1);
    }
}

int DepthImageObstacleDetector::extract_blobs()
{
    // Check if the current stored depth frame is valid
    if (depth_frame.size() == 0) {
        return 0;
    }

    // Instantiate disjoint data set
    PPTree ds_tree(MAX_NUM_BLOBS);

    // Store number of pixels for each blob
    int blob_num_pixels[MAX_NUM_BLOBS] = {0};

    // Blob to Obstacle Vector
    int blob_to_obstacle[MAX_NUM_BLOBS];
    int num_obstacles = 0;
    int curr_label = 1;

    // Init Obstacles vector and labels vector
    init_obstacle_array(obstacles);
    this->labels.resize(depth_frame.size());

    // First Pass
    for (int i = 0; i < this->height; i++) {
        for (int j = 0; j < this->width; j++) {
            if (this->depth_frame[i * this->width + j] != BACKGROUND) {
                std::vector<int> neigh_labels(4);

                int num_neighbors = get_neighbors_label(i, j, neigh_labels.data());

                if (num_neighbors) {
                    this->labels[i * this->width + j] = neigh_labels[0];
                    for (int k = 1; k < num_neighbors; k++) {
                        ds_tree.ds_union(neigh_labels[0], neigh_labels[k]);
                    }
                } else {
                    if (curr_label <= MAX_NUM_BLOBS) {
                        this->labels[i * this->width + j] =
                            curr_label;
                        curr_label++;
                    } else {
                        this->labels[i * this->width + j] = 0;
                    }
                }
            } else {
                this->labels[i * this->width + j] = 0;
            }
        }
    }

    /* Second Pass. */
    for (unsigned int i = 0; i < labels.size(); i++) {
        if (this->labels[i]) {
            this->labels[i] = ds_tree.ds_find(this->labels[i]);
            blob_num_pixels[this->labels[i]]++;
        }
    }

    /* Initialize blob to obstacle vector. */
    for (int i = 0; i < MAX_NUM_BLOBS; i++) {
        blob_to_obstacle[i] = -1;
    }

    /* Third Pass */
    for (int i = 0; i < this->height; i++) {
        for (int j = 0; j < this->width; j++) {
            int index = this->labels[i * this->width + j];

            if (this->depth_frame[index] == BACKGROUND) {
                continue;
            }

            if (blob_num_pixels[index] < this->min_num_pixels) {
                if (j > 0) {
                    this->labels[index] = this->labels[index - 1];
                } else {
                    this->labels[index] = 0;
                }
                continue;
            }

            if (blob_to_obstacle[index] == -1 &&
                num_obstacles < this->max_num_obstacles) {
                blob_to_obstacle[index] = num_obstacles;
                obstacles[blob_to_obstacle[index]].id = num_obstacles + 1;
                num_obstacles++;
            }

            if (blob_to_obstacle[this->labels[i * this->width + j]] != -1) {
                /* Calculate obstacles parameters. */

                obstacles[blob_to_obstacle[index]].center =
                    glm::dvec3(j, i, depth_frame[i * this->width + j]);
            }
        }
    }

    return num_obstacles;
}

