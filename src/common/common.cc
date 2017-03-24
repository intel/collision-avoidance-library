/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>

#include "common/common.hh"
#include "common/math.hh"

Pose operator-(const Pose &a, const Pose &b)
{
    glm::dquat tmp = {0.0, a.pos.x - b.pos.x, a.pos.y - b.pos.y,
                      a.pos.z - b.pos.z};
    tmp = glm::inverse(b.rot) * (tmp * b.rot);

    return Pose{{tmp.x, tmp.y, tmp.z},
                glm::normalize(glm::inverse(b.rot) * a.rot)};
}

void Pose::set_rot(float pitch, float roll, float yaw)
{
    double phi, the, psi;

    phi = roll / 2.0;
    the = pitch / 2.0;
    psi = yaw / 2.0;

    this->rot.w =
        cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
    this->rot.x =
        sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    this->rot.y =
        cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    this->rot.z =
        cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
}

double Pose::pitch()
{
    // GLM uses an unusual convention to convert quaternions to euler angles
    // with values swapped
    return glm::yaw(rot);
}

double Pose::roll()
{
    // GLM uses an unusual convention to convert quaternions to euler angles
    // with values swapped
    return glm::pitch(rot);
}

double Pose::yaw()
{
    // GLM uses an unusual convention to convert quaternions to euler angles
    // with values swapped
    return glm::roll(rot);
}
