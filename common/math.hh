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
 * @brief Custom math classes and methods
 */

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

/**
 * @brief Stores a position vector and a rotation quaternion in three
 *        dimensional space.
 */
struct Pose {
    glm::dvec3 pos;
    glm::dquat rot;

    /**
     * @brief Rotation about the y-axis.
     * @return The rotation in radians.
     */
    double pitch();

    /**
     * @brief Return the rotation about the x-axis in radians.
     * @return The rotation in radians.
     */
    double roll();

    /**
     * @brief Return the rotation about the z-axis in radians.
     * @return The rotation in radians.
     */
    double yaw();

    /**
     * @brief Set the pose rotation with euler angles.
     */
    void set_rot(float pitch, float roll, float yaw);
};

/**
 * @brief Pose substraction
 */
Pose operator-(const Pose& a, const Pose &b);

/**
 * @brief Get the signal of x.
 * @return -1 if x is negative, +1 if x is positive and 0 if x equals
 *        zero.
 */
int sign(double x);

/**
 * @brief Sigmoid function.
 * @return The sigmoid of x.
 */
double sigmoid(double x);

/**
 * @brief Stores a polar position vector in three dimensional space.
 */
typedef struct {
    double len;
    double theta; // azimuthal angle
    double phi;   // polar angle
} PolarVector;

/**
 * @brief Convert cartesian coordinates to spherical coordinates.
 */
PolarVector cartesian_to_spherical(double x, double y, double z);

/**
 * @brief Check if the value X is within A and B (inclusive).
 */
#define inbounds(X, A, B) ((X) >= A && (X) <= B)

