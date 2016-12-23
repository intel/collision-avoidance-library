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

#include <glm/glm.hpp>
#include <iostream>
#include <mutex>

#include "RealSenseCamera.hh"
#include "utils/DebugUtils.hh"

RealSenseCamera::RealSenseCamera(size_t width, size_t height, unsigned int fps) try
{
    this->width = width;
    this->height = height;

    this->ctx = std::make_shared<rs::context>();
    if (this->ctx->get_device_count() == 0) {
        std::cerr << "[RealSenseCamera] No RealSense device found." << std::endl;
        return;
    }

    this->dev = this->ctx->get_device(0);
    this->dev->enable_stream(rs::stream::depth, this->width, this->height,
            rs::format::z16, fps);

    auto intrinsics = this->dev->get_stream_intrinsics(rs::stream::depth);
    this->hfov = glm::radians(intrinsics.hfov());
    this->vfov = glm::radians(intrinsics.vfov());

    this->scale = this->dev->get_depth_scale();

    this->dev->start();

    this->depth_buffer.resize(this->width * this->height);
}
catch(const rs::error &e)
{
    std::cerr << "[RealSenseCamera] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;
}

std::vector<uint16_t> &RealSenseCamera::get_depth_buffer() try
{
    if (this->dev == nullptr) {
        std::cerr << "[RealSenseCamera] Error: No device available." << std::endl;
        return this->depth_buffer;
    }

    this->dev->wait_for_frames();

    const uint16_t *frame = reinterpret_cast<const uint16_t *>
        (this->dev->get_frame_data(rs::stream::depth));

    std::lock_guard<std::mutex> locker(depth_buffer_mtx);
    for (size_t i = 0; i < this->width * this->height; i++)
        this->depth_buffer[i] = frame[i];

    if (this->visualization_on)
        this->visualize();

    return this->depth_buffer;
}
catch(const rs::error &e)
{
    std::cerr << "[RealSenseCamera] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;

    return this->depth_buffer;
}

double RealSenseCamera::get_horizontal_fov()
{
    return hfov;
}

double RealSenseCamera::get_vertical_fov()
{
    return vfov;
}

void RealSenseCamera::visualization(bool onoff)
{
    if (!onoff) {
        this->visualization_on = false;
        glfwDestroyWindow(this->win);
        glfwTerminate();
        delete this->frame_buffer;
        return;
    }

    if (!this->win) {
        glfwInit();
        this->win = glfwCreateWindow(this->width, this->height, "RealSense Depth Buffer", 0, 0);
        this->frame_buffer = new uint8_t[this->width * this->height * 3];

        glGenTextures(1, &this->texture);
        this->visualization_on = true;
    }

    printf("Scale:\n");
    printf("null  : BLACK\n");
    printf("0.01 m: RED\n");
    printf("1.25 m: YELLOW\n");
    printf("2.50 m: GREEN\n");
    printf("3.75 m: CYAN\n");
    printf("5.0+ m: BLUE\n");
}

void RealSenseCamera::visualize(void)
{
    glfwMakeContextCurrent(this->win);

    glViewport(0, 0, this->width, this->height);
    glClear(GL_COLOR_BUFFER_BIT);

    glPushMatrix();
    glOrtho(0, this->width, this->height, 0, -1, +1);

    glBindTexture(GL_TEXTURE_2D, this->texture);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, this->width);

    for (unsigned int i = 0; i < this->depth_buffer.size(); i++) {
        uint8_t *rgb = this->frame_buffer + (3 * i);
        rainbow_scale(((double)this->depth_buffer[i] * this->scale) / 5.0, rgb);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, this->width, this->height, 0, GL_RGB,
        GL_UNSIGNED_BYTE, reinterpret_cast<const GLvoid *>(this->frame_buffer));

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glBindTexture(GL_TEXTURE_2D, this->texture);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);

    // Order matters :/
    glTexCoord2f(0, 0); glVertex2f(0, 0);
    glTexCoord2f(1, 0); glVertex2f(this->width, 0);
    glTexCoord2f(1, 1); glVertex2f(this->width, this->height);
    glTexCoord2f(0, 1); glVertex2f(0, this->height);

    glEnd();
    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);

    glPopMatrix();
    glfwSwapBuffers(this->win);
}
