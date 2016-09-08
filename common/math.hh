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

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

struct Pose {
    glm::dvec3 pos;
    glm::dquat rot;

    double pitch();
    double roll();
    double yaw();
    void set_rot(float pitch, float roll, float yaw);
};

Pose operator-(const Pose& a, const Pose &b);

int sign(double x);
double sigmoid(double x);

typedef struct {
    double len;
    double theta; // azimuthal angle
    double phi;   // polar angle
} PolarVector;

PolarVector cartesian_to_spherical(double x, double y, double z);

#define inbounds(X, A, B) ((X) >= A && (X) <= B)

