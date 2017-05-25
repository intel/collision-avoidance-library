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

#include "Sensors.hh"

#include <cmath>

unsigned int DepthCamera::get_height()
{
    return height;
}

unsigned int DepthCamera::get_width()
{
    return width;
}

double DepthCamera::get_scale()
{
    return scale;
}

std::shared_ptr<struct DepthData> DepthCamera::read()
{
    std::shared_ptr<struct DepthData> data = std::make_shared<struct DepthData>();

    data->height = this->get_height();
    data->width = this->get_width();
    data->scale = this->get_scale();
    data->hfov = this->get_horizontal_fov();
    data->vfov = this->get_vertical_fov();
    data->depth_buffer = this->get_depth_buffer();

    return data;
}
