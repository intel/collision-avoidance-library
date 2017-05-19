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

#ifdef WITH_VDEBUG

#include <memory>

#include <GL/glu.h>

#include "visual.hh"

VisualDepth::VisualDepth(int x, int y, unsigned int width, unsigned int height)
{
    this->set_viewport(x, y, width, height);

    this->frame_buffer = nullptr;
    this->frame_buffer_size = 0;

    glGenTextures(1, &this->texture);
}

void VisualDepth::set_viewport(int x, int y, unsigned int width, unsigned int height)
{
    this->x = x;
    this->y = y;
    this->width = width;
    this->height = height;
}

VisualDepth::~VisualDepth()
{
    delete this->frame_buffer;
}

void VisualDepth::info()
{
    cout << "Scale:" << endl;
    cout << "null  : BLACK" << endl;
    cout << "0.01 m: RED" << endl;
    cout << "1.25 m: YELLOW" << endl;
    cout << "2.50 m: GREEN" << endl;
    cout << "3.75 m: CYAN" << endl;
    cout << "5.0+ m: BLUE" << endl;
}

void VisualDepth::rainbow_scale(double value, uint8_t rgb[])
{
    rgb[0] = rgb[1] = rgb[2] = 0;

    if (value <= 0.0)
        return;

    if (value < 0.25) { // RED to YELLOW
        rgb[0] = 255;
        rgb[1] = (uint8_t)255 * (value / 0.25);
    } else if (value < 0.5) { // YELLOW to GREEN
        rgb[0] = (uint8_t)255 * (1 - ((value - 0.25) / 0.25));
        rgb[1] = 255;
    } else if (value < 0.75) { // GREEN to CYAN
        rgb[1] = 255;
        rgb[2] = (uint8_t)255 * (value - 0.5 / 0.25);
    } else if (value < 1.0) { // CYAN to BLUE
        rgb[1] = (uint8_t)255 * (1 - ((value - 0.75) / 0.25));
        rgb[2] = 255;
    } else { // BLUE
        rgb[2] = 255;
    }
}

void VisualDepth::visualize(shared_ptr<DepthData> depth_data)
{
    if (depth_data == nullptr) {
        return;
    }

    if (this->frame_buffer_size < depth_data->width * depth_data->height * 3) {
        delete this->frame_buffer;
        this->frame_buffer = new uint8_t[depth_data->width * depth_data->height * 3];
    }

    glViewport(this->x, this->y, this->width, this->height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, 1, 0, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f);

    glColor3f(1.0f, 1.0f, 1.0f);

    glBindTexture(GL_TEXTURE_2D, this->texture);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, depth_data->width);

    for (unsigned int i = 0; i < depth_data->depth_buffer.size(); i++) {
        uint8_t *rgb = this->frame_buffer + (3 * i);
        this->rainbow_scale(((double)depth_data->depth_buffer[i] * depth_data->scale) / 5.0, rgb);
    }

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, depth_data->width, depth_data->height, 0, GL_RGB,
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
        glTexCoord2f(0, 0);
        glVertex2f(0, 1);

        glTexCoord2f(1, 0);
        glVertex2f(1, 1);

        glTexCoord2f(1, 1);
        glVertex2f(1, 0);

        glTexCoord2f(0, 1);
        glVertex2f(0, 0);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}

#endif
