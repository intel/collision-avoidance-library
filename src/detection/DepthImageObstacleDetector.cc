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
#include <cstdlib>
#include <vector>

#include "DepthImageObstacleDetector.hh"
#include "common/common.hh"

#define MAX_NUM_LABELS UINT16_MAX
#define BACKGROUND 0

struct Set {
    Set *parent;
    int rank = 0;

    Set();
    Set *repr();
    static void join(Set *a, Set *b);
};

class PPTree
{
  public:
    PPTree(int num_nodes);
    ~PPTree();

    void ds_union(int node_x_id, int node_y_id);
    int ds_find(int node_id);

  private:
    Set *find(int node_id);
    std::vector<Set> sets;
};

Set::Set()
    : parent(this)
{
}

Set *Set::repr()
{
    if (this->parent == this)
        return this;

    return this->parent = this->parent->repr();
}

PPTree::PPTree(int num_nodes)
    : sets(num_nodes)
{
}

int PPTree::ds_find(int node_id)
{
    return this->find(node_id) - this->sets.data();
}

Set *PPTree::find(int node_id)
{
    return this->sets[node_id].repr();
}


void PPTree::ds_union(int a, int b)
{
    Set::join(&this->sets[a], &this->sets[b]);
}

PPTree::~PPTree()
{
}

void Set::join(Set *a, Set *b)
{
    a = a->repr();
    b = b->repr();

    if (a == b) {
        /* Already united. */
        return;
    }

    /* Unite the roots of the nodes. */
    if (a->rank < b->rank) {
        a->parent = b;
        return;
    }

    b->parent = a;
    if (a->rank == b->rank)
        a->rank += 1;
}

DepthImageObstacleDetector::DepthImageObstacleDetector(
    std::shared_ptr<DepthCamera> depth_camera, double threshold_meters)
{
    this->sensor = depth_camera;
    this->obstacles.resize(MAX_NUM_LABELS);
    this->threshold = threshold_meters;
}

const std::vector<Obstacle> &DepthImageObstacleDetector::detect()
{
    // Store current camera frame data
    this->depth_frame = this->sensor->get_depth_buffer();
    this->height = this->sensor->get_height();
    this->width = this->sensor->get_width();
    this->scale = this->sensor->get_scale();
    this->hfov = this->sensor->get_horizontal_fov();
    this->vfov = this->sensor->get_vertical_fov();
    this->base_phi = (M_PI - hfov) / 2;
    this->base_theta = (M_PI - vfov) / 2;

    // Detect obstacles from current depth buffer
    this->extract_blobs();

    return this->obstacles;
}

inline bool DepthImageObstacleDetector::is_valid(const uint16_t depth)
{
    uint16_t threshold_scaled = (uint16_t) (this->threshold / this->scale);
    return (depth != BACKGROUND && (threshold_scaled ? depth < threshold_scaled : true));
}

inline bool DepthImageObstacleDetector::is_in_range(const uint16_t d1, const uint16_t d2)
{
    return (abs(d1 - d2) <= this->tolerance);
}

int DepthImageObstacleDetector::get_neighbors_label(const int i, const int j, std::vector<int> &neigh_labels)
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
        if (is_in_range(depth_frame[pixel_idx - 1], depth_frame[pixel_idx])) {
            neigh_labels[neighbor_idx] = this->labels[pixel_idx - 1];
            neighbor_idx++;
        }
    }

    /* Check northwest */
    if (j > 0 && i > 0) {
        if (is_in_range(depth_frame[north_idx - 1], depth_frame[pixel_idx])) {
            neigh_labels[neighbor_idx] = this->labels[north_idx - 1];
            neighbor_idx++;
        }
    }

    /* Check north */
    if (i > 0) {
        if (is_in_range(depth_frame[north_idx], depth_frame[pixel_idx])) {
            neigh_labels[neighbor_idx] = this->labels[north_idx];
            neighbor_idx++;
        }
    }

    /* Check northeast */
    if (i > 0 && j < (this->width - 1)) {
        if (is_in_range(depth_frame[north_idx + 1], depth_frame[pixel_idx])) {
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
    obstacles.resize(MAX_NUM_LABELS);
    for (unsigned int i = 0; i < obstacles.size(); ++i) {
        obstacles[i].id = -1;
        obstacles[i].center = glm::dvec3(-1, -1, -1);
    }
}

int DepthImageObstacleDetector::extract_blobs()
{
    int row_offset;

    // Check if the current stored depth frame is valid
    if (depth_frame.size() == 0) {
        return 0;
    }

    // Instantiate disjoint data set
    PPTree ds_tree(MAX_NUM_LABELS);

    // Store number of pixels for each blob
    int blob_num_pixels[MAX_NUM_LABELS] = {0};

    // Blob to Obstacle Vector
    int blob_to_obstacle[MAX_NUM_LABELS];
    int num_obstacles = 0;
    uint16_t curr_label = 1;

    // Init Obstacles vector and labels vector
    init_obstacle_array(obstacles);
    this->labels.resize(depth_frame.size());

    // First Pass
    for (int i = 0; i < this->height; i++) {
        row_offset = i * this->width;
        for (int j = 0; j < this->width; j++) {
            if (is_valid(this->depth_frame[row_offset + j])) {
                std::vector<int> neigh_labels(4);

                int num_neighbors = get_neighbors_label(i, j, neigh_labels);
                if (num_neighbors) {
                    this->labels[row_offset + j] = neigh_labels[0];
                    for (int k = 1; k < num_neighbors; k++) {
                        ds_tree.ds_union(neigh_labels[0], neigh_labels[k]);
                    }
                } else {
                    this->labels[row_offset + j] = (curr_label <= MAX_NUM_LABELS) ? curr_label++ : 0;
                }
            } else {
                this->labels[row_offset + j] = 0;
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
    for (int i = 0; i < MAX_NUM_LABELS; i++) {
        blob_to_obstacle[i] = -1;
    }

    /* Third Pass */

    for (int i = 0; i < this->height; i++) {
        row_offset = i * this->width;
        for (int j = 0; j < this->width; j++) {
            int label = this->labels[row_offset + j];

            if (!label || blob_num_pixels[label] < this->min_num_pixels)
                continue;

            if (blob_to_obstacle[label] == -1) {
                if (num_obstacles >= this->max_num_obstacles)
                    continue;

                blob_to_obstacle[label] = num_obstacles++;
                obstacles[blob_to_obstacle[label]].id = label;
                obstacles[blob_to_obstacle[label]].center.x = DBL_MAX;
            }

            Obstacle *o = &obstacles[blob_to_obstacle[label]];
            o->center += glm::dvec3(0, i, j);
            o->center.x = (depth_frame[row_offset + j] < o->center.x) ?
                depth_frame[row_offset + j] : o->center.x;
        }
    }

    this->obstacles.resize(num_obstacles);
    for (Obstacle &o : obstacles) {
        o.center.x *= this->scale;
        o.center.y /= blob_num_pixels[o.id];
        o.center.z /= blob_num_pixels[o.id];

        // Cartesian to spherical
        o.center.y = ((o.center.y / this->height) * vfov) + base_theta;
        o.center.z = ((1.0 - (o.center.z / this->width)) * hfov) + base_phi;
    }

    return num_obstacles;
}

