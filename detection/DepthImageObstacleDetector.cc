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

DepthImageObstacleDetector::DepthImageObstacleDetector(std::shared_ptr<DepthCamera> _depth_camera)
    : depth_camera(_depth_camera)
{
    this->obstacles.resize(MAX_NUM_BLOBS);
}

DepthImageObstacleDetector::~DepthImageObstacleDetector()
{
}

const std::vector<Obstacle> &DepthImageObstacleDetector::detect()
{
    // Store current camera frame data
    this->curr_depth_frame = this->depth_camera->get_depth_buffer();
    this->curr_height = this->depth_camera->get_height();
    this->curr_width = this->depth_camera->get_width();
    this->curr_scale = this->depth_camera->get_scale();
    this->curr_fov = this->depth_camera->get_fov_tan();

    // Detect obstacles from current depth buffer
    int num_obstacles;
    num_obstacles = this->extract_blobs();

    // Return a copy of the obstacles with the correct size
    std::vector<Obstacle> resized_obstacles(this->obstacles);
    resized_obstacles.resize(num_obstacles);

    return resized_obstacles;
}

int DepthImageObstacleDetector::get_neighbors_label(int i, int j, int *neigh_labels)
{
    int neighbor_idx = 0;

    if (i < 0 || i >= this->curr_height || j < 0 || j >= this->curr_width) {
        return 0;
    }

    /* Check west */
    if (j > 0) {
        if (fabs(curr_depth_frame[i * this->curr_width + (j - 1)] -
                 curr_depth_frame[i * this->curr_width + j]) <=
            color_tolerance) {
            neigh_labels[neighbor_idx] =
                this->curr_labels[i * this->curr_width + (j - 1)];
            neighbor_idx++;
        }
    }

    /* Check northwest */
    if (j > 0 && i > 0) {
        if (fabs(curr_depth_frame[(i - 1) * this->curr_width + (j - 1)] -
                 curr_depth_frame[i * this->curr_width + j]) <=
            color_tolerance) {
            neigh_labels[neighbor_idx] =
                this->curr_labels[(i - 1) * this->curr_width + (j - 1)];
            neighbor_idx++;
        }
    }

    /* Check north */
    if (i > 0) {
        if (fabs(curr_depth_frame[(i - 1) * this->curr_width + j] -
                 curr_depth_frame[i * this->curr_width + j]) <=
            color_tolerance) {
            neigh_labels[neighbor_idx] =
                this->curr_labels[(i - 1) * this->curr_width + j];
            neighbor_idx++;
        }
    }

    /* Check northeast */
    if (i > 0 && j < (this->curr_width - 1)) {
        if (fabs(curr_depth_frame[(i - 1) * this->curr_width + j + 1] -
                 curr_depth_frame[i * this->curr_width + j]) <=
            color_tolerance) {
            neigh_labels[neighbor_idx] =
                this->curr_labels[(i - 1) * this->curr_width + j + 1];
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
    for (unsigned int i = 0; i < obstacles.size(); ++i) {
        obstacles[i].id = -1;
        obstacles[i].center = glm::dvec3(-1, -1, -1);
    }
}

int DepthImageObstacleDetector::extract_blobs()
{
    // Check if the current stored depth frame is valid
    if (curr_depth_frame.size() == 0) {
        return 0;
    }

    // Instantiate disjoint data set
    PPTree ds_tree(MAX_NUM_BLOBS);

    // Store number of pixels for each blob
    int blob_num_pixels[MAX_NUM_BLOBS] = {0};

    // Blob to Obstacle HashTable
    int blob_to_obstacle[MAX_NUM_BLOBS];
    int num_obstacles = 0;
    int curr_label = 1;

    // Init Obstacles vector and labels vector
    init_obstacle_array(obstacles);
    this->curr_labels.resize(curr_depth_frame.size());

    // First Pass
    for (int i = 0; i < this->curr_height; i++) {
        for (int j = 0; j < this->curr_width; j++) {
            if (this->curr_depth_frame[i * this->curr_width + j] !=
                this->bg_color) {
                std::vector<int> neigh_labels(4);
                int num_neighbors = get_neighbors_label(i, j, neigh_labels.data());
                if (num_neighbors) {
                    this->curr_labels[i * this->curr_width + j] =
                        neigh_labels[0];
                    for (int k = 1; k < num_neighbors; k++) {
                        ds_tree.ds_union(neigh_labels[0], neigh_labels[k]);
                    }
                } else {
                    if (curr_label <= MAX_NUM_BLOBS) {
                        this->curr_labels[i * this->curr_width + j] =
                            curr_label;
                        curr_label++;
                    } else {
                        this->curr_labels[i * this->curr_width + j] = 0;
                    }
                }
            } else {
                this->curr_labels[i * this->curr_width + j] = 0;
            }
        }
    }

    /* Second Pass. */
    for (int i = 0; i < this->curr_height; i++) {
        for (int j = 0; j < this->curr_width; j++) {
            if (this->curr_depth_frame[i * this->curr_width + j] !=
                this->bg_color) {
                this->curr_labels[i * this->curr_width + j] = ds_tree.ds_find(
                    this->curr_labels[i * this->curr_width + j]);
                blob_num_pixels[this->curr_labels[i * this->curr_width + j]]++;
            }
        }
    }

    /* Initialize blob to obstacle hashtable. */
    for (int i = 0; i < MAX_NUM_BLOBS; i++) {
        blob_to_obstacle[i] = -1;
    }

    /* Third Pass */
    for (int i = 0; i < this->curr_height; i++) {
        for (int j = 0; j < this->curr_width; j++) {
            int index = this->curr_labels[i * this->curr_width + j];

            if (this->curr_depth_frame[index] ==
                this->bg_color) {
                continue;
            }

            if (blob_num_pixels[index] < this->min_num_pixels) {
                if (j > 0) {
                    this->curr_labels[index] = this->curr_labels[index - 1];
                } else {
                    this->curr_labels[index] = 0;
                }
                continue;
            }

            if (blob_to_obstacle[index] == -1 &&
                num_obstacles < this->max_num_obstacles) {
                blob_to_obstacle[index] = num_obstacles;
                obstacles[blob_to_obstacle[index]].id = num_obstacles + 1;
                num_obstacles++;
            }

            if (blob_to_obstacle[this->curr_labels[i * this->curr_width + j]] != -1) {
                /* Calculate obstacles parameters. */

                obstacles[blob_to_obstacle[index]].center =
                    glm::dvec3(j, i, curr_depth_frame[i * this->curr_width + j]);
            }
        }
    }

    return num_obstacles;
}

