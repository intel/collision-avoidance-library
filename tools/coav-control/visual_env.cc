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

#ifdef WITH_VDEBUG

#include <memory>
#include <glm/glm.hpp>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <coav/coav.hh>

#include "visual.hh"

void draw_blade(float x, float y, float z)
{
    glPushMatrix();

    glColor3f(0.4f, 0.4f, 0.4f);

    GLUquadricObj *cylinder;
    cylinder = gluNewQuadric();

    GLUquadricObj *circle;
    circle = gluNewQuadric();

    glTranslatef(x, y, z);
    gluCylinder(cylinder, 0.3f, 0.3f, 0.05f, 32, 32);
    gluDisk(circle, 0.0, 0.3f, 32, 1);
    glTranslatef(0.0f, 0.0f, 0.05f);
    gluDisk(circle, 0.0, 0.3f, 32, 1);

    glPopMatrix();
}

void draw_vehicle()
{
    glPushMatrix();

    glColor3f(0.1f, 0.1f, 0.1f);

    // top
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(-0.2f, 0.3f, 0.2f);
    glVertex3f(0.2f, 0.3f, 0.2f);
    glVertex3f(0.2f, -0.3f, 0.2f);
    glVertex3f(-0.2f, -0.3f, 0.2f);

    glEnd();

    // front
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.2f, 0.3f, 0.2f);
    glVertex3f(0.2f, 0.3f, 0.0f);
    glVertex3f(-0.2f, 0.3f, 0.0f);
    glVertex3f(-0.2f, 0.3f, 0.2f);

    glEnd();

    // right
    glBegin(GL_QUADS);
    glNormal3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.2f, 0.3f, 0.0f);
    glVertex3f(0.2f, 0.3f, 0.2f);
    glVertex3f(0.2f, -0.3f, 0.2f);
    glVertex3f(0.2f, -0.3f, 0.0f);

    glEnd();

    // left
    glBegin(GL_QUADS);
    glNormal3f(-1.0f, 0.0f, 0.0f);
    glVertex3f(-0.2f, -0.3f, 0.2f);
    glVertex3f(-0.2f, 0.3f, 0.2f);
    glVertex3f(-0.2f, 0.3f, 0.0f);
    glVertex3f(-0.2f, -0.3f, 0.0f);

    glEnd();

    // bottom
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, -1.0f);
    glVertex3f(0.2f, 0.3f, 0.0f);
    glVertex3f(0.2f, -0.3f, 0.0f);
    glVertex3f(-0.2f, -0.3f, 0.0f);
    glVertex3f(-0.2f, 0.3f, 0.0f);

    glEnd();

    // back
    glBegin(GL_QUADS);
    glNormal3f(0.0f, -1.0f, 0.0f);
    glVertex3f(0.2f, -0.3f, 0.2f);
    glVertex3f(0.2f, -0.3f, 0.0f);
    glVertex3f(-0.2f, -0.3f, 0.0f);
    glVertex3f(-0.2f, -0.3f, 0.2f);

    glEnd();

    draw_blade(0.4, 0.4, 0.2);
    draw_blade(0.4, -0.4, 0.2);
    draw_blade(-0.4, -0.4, 0.2);
    draw_blade(-0.4, 0.4, 0.2);

}

void draw_obstacle(float x, float y, float z)
{
    glPushMatrix();

    glTranslatef(x, y, z);
    glColor3f(0.7f, 0.0f, 0.0f);
    glutSolidSphere(0.3, 20.0, 20.0);

    glPopMatrix();
}

void draw_grid()
{
    glPushMatrix();

    glBegin(GL_LINES);
        glColor3f(0.6f, 0.6f, 0.6f);
        for (int i = -10; i <= 10; i++) {
            if (i == 0) continue;

            glNormal3f(0.0f, 0.0f, 1.0f);
            glVertex3f(100, i * 10, 0);
            glVertex3f(-100, i * 10, 0);

            glNormal3f(0.0f, 0.0f, 1.0f);
            glVertex3f(i * 10, 100, 0);
            glVertex3f(i * 10, -100, 0);
        }

        // X axis
        glColor3f(0.8, 0.0, 0.0);
        glVertex3f(-100, 0, 0);
        glVertex3f( 100, 0, 0);
        // Y axis
        glColor3f(0.0, 0.8, 0.0);
        glVertex3f(0, -100, 0);
        glVertex3f(0,  100, 0);
        // Z axis
        glColor3f(0.0, 0.0, 0.8);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 100);
    glEnd();

    glTranslatef(0.0, 0.0, -0.01);

    glBegin(GL_QUADS);
    glNormal3f(0.0f, 0.0f, 1.0f);
    glColor3f(0.5f, 0.5f, 0.5f);
    glVertex3f(100.0f, 100.0f, 0.0f);
    glVertex3f(100.0f, -100.0f, 0.0f);
    glVertex3f(-100.0f, -100.0f, 0.0f);
    glVertex3f(-100.0f, 100.0f, 0.0f);
    glEnd();

    glPopMatrix();
}

VisualEnvironment::VisualEnvironment(int x, int y, unsigned int width, unsigned int height)
{
    this->set_viewport(x, y, width, height);
}

void VisualEnvironment::set_viewport(int x, int y, unsigned int width, unsigned int height)
{
    this->x = x;
    this->y = y;
    this->width = width;
    this->height = height;
}

void VisualEnvironment::visualize(shared_ptr<MavQuadCopter> vehicle, vector<Obstacle> obstacles)
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat position[] = { 1.0f, 1.0f, 0.0f, 0.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glViewport(this->x, this->y, this->width, this->height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, float(this->width) / float(this->height), 1, 1000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(this->x_eye, this->y_eye, this->z_eye, 0, 0, 0, 0, 0, 1);

    draw_grid();

    glm::dvec3 pos = vehicle->vehicle_pose().pos;
    glm::dquat rot = vehicle->vehicle_pose().rot;

    glPushMatrix();

    glTranslatef(pos.x, pos.y, pos.z);
    auto angleAxis = glm::axis(rot);
    glRotatef(glm::angle(rot), angleAxis.x, angleAxis.y, angleAxis.z);

    draw_vehicle();

    for (Obstacle o : obstacles) {
        float r = o.center.x;
        float theta = o.center.y;
        float phi = o.center.z;

        float ox = r * sin(theta) * cos(phi);
        float oy = r * sin(theta) * sin(phi);
        float oz = r * cos(theta);

        draw_obstacle(ox, oy, oz);
    }

    glPopMatrix();

    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_DEPTH_TEST);
}

void VisualEnvironment::on_mouse_move(int x, int y)
{
    double angle = 3.14 * float(this->x_enter + x) / this->width;
    this->x_eye = this->zoom * sin(angle);
    this->y_eye = this->zoom * cos(angle);
    this->z_eye = (this->y_enter + y) / 10 ;

    glutPostRedisplay();
}

void VisualEnvironment::on_mouse_button(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            this->x_enter = this->x_enter - x;
            this->y_enter = this->y_enter - y;
        } else if (state == GLUT_UP) {
            this->x_enter = this->x_enter + x;
            this->y_enter = this->y_enter + y;
        }
    } else if (button == 3) {
        if (state == GLUT_UP) return;
        this->zoom -= 5;
        this->x_eye = this->zoom / (this->zoom + 5) * this->x_eye;
        this->y_eye = this->zoom / (this->zoom + 5) * this->y_eye;
    } else if (button == 4) {
        if (state == GLUT_UP) return;
        this->zoom += 5;
        this->x_eye = this->zoom / (this->zoom - 5) * this->x_eye;
        this->y_eye = this->zoom / (this->zoom - 5) * this->y_eye;
    }

    glutPostRedisplay();
}

#endif
