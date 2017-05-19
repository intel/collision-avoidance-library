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

#pragma once

#include <memory>

#include <coav/coav.hh>

#include <GL/gl.h>

using namespace std;

class VisualDepth
{
public:
    VisualDepth(int x, int y, unsigned int width, unsigned int height);
    ~VisualDepth();
    void info();
    void rainbow_scale(double value, uint8_t rgb[]);
    void visualize(shared_ptr<DepthData> depth_data, vector<Obstacle> obstacles = vector<Obstacle>());
    void set_viewport(int x, int y, unsigned int width, unsigned int height);

private:
    int x;
    int y;

    unsigned int width;
    unsigned int height;

    GLuint texture;
    uint8_t *frame_buffer;
    size_t frame_buffer_size;
};

class VisualEnvironment
{
public:
    VisualEnvironment(int x, int y, unsigned int width, unsigned int height);
    void visualize(shared_ptr<MavQuadCopter> vehicle, vector<Obstacle> obstacles);
    void set_viewport(int x, int y, unsigned int width, unsigned int height);

    void on_mouse_move(int mouseX, int mouseY);
    void on_mouse_button(int button, int state, int mouseX, int mouseY);

private:
    int x;
    int y;

    unsigned int width;
    unsigned int height;

    double x_eye = 40;
    double y_eye = -30;
    double z_eye = 15;
    double zoom = 20;

    int x_enter = 0;
    int y_enter = 0;
};

struct VisualData
{
    struct {
        unsigned int width;
        unsigned int height;

        int ref;
    } window;

    struct {
        shared_ptr<MavQuadCopter> vehicle;
        shared_ptr<DepthCamera> sensor;
        shared_ptr<Detector> detector;
        shared_ptr<CollisionAvoidanceStrategy<MavQuadCopter>> avoidance;

        shared_ptr<DepthData> depth_data;
        vector<Obstacle> obstacles;
    } coav;

    shared_ptr<VisualDepth> depth;
    shared_ptr<VisualEnvironment> env;
};

void visual_mainlopp(int argc, char* argv[], shared_ptr<MavQuadCopter> vehicle,
        shared_ptr<DepthCamera> sensor, shared_ptr<Detector> detector,
        shared_ptr<CollisionAvoidanceStrategy<MavQuadCopter>> avoidance);
