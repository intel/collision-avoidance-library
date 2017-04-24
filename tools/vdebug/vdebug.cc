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

#include <memory>
#include <iostream>
#include <cmath>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <coav/coav.hh>

auto gzrs_camera = std::make_shared<GazeboRealSenseCamera>();
auto detector = std::make_shared<DepthImageObstacleDetector>(gzrs_camera);
auto gz_quad = std::make_shared<MavQuadCopter>();
std::vector<Obstacle> obstacles;

static const int windowWidth = 1000;
static const int windowHeight = 1000;
static double eyeX = 0;
static double eyeY = 0;
static double eyeZ = 0;

static int enterX = 0, enterY = 0;

void mouseMove(int mouseX, int mouseY)
{
    double angle = 3.14 * float(enterX + mouseX) / windowWidth;
    eyeX = 100 * sin(angle);
    eyeY = 100 * cos(angle);
    eyeZ = (enterY + mouseY) / 10 ;

    glutPostRedisplay();
}

void updateMouse(int button, int state, int mouseX, int mouseY)
{
    if (button != GLUT_LEFT_BUTTON)
        return;

    if (state == GLUT_DOWN) {
        enterX = enterX - mouseX;
        enterY = enterY - mouseY;
    } else if (state == GLUT_UP) {
        enterX = enterX + mouseX;
        enterY = enterY + mouseY;
    }
}

void updateDisplay()
{
    glClearColor(0.1, 0.1, 0.1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset transformations
    glLoadIdentity();
    glScalef(-1, 1, 1);
    glOrtho(-10, 10, -10, 10, -100, 1000);
    gluLookAt(eyeX, eyeY, eyeZ,
        0, 0, 0,
        0, 0, 1);

    glEnable(GL_FOG);
    glFogf(GL_FOG_DENSITY, 1.0f);
    glFogf(GL_FOG_START, 20.0f);
    glFogf(GL_FOG_END, 120.0f);

    glBegin(GL_QUADS);
        glColor3f(0.2, 0.2, 0.2);
        glVertex3f(-100, -100, -0.01);
        glVertex3f(-100,  100, -0.01);
        glVertex3f( 100,  100, -0.01);
        glVertex3f( 100, -100, -0.01);
    glEnd();

    glBegin(GL_LINES);
        // X axis
        glColor3f(1.0, 0, 0);
        glVertex3f(-100, 0, 0);
        glVertex3f( 100, 0, 0);
        // Y axis
        glColor3f(0, 1.0, 0);
        glVertex3f(0, -100, 0);
        glVertex3f(0,  100, 0);
        // Z axis
        glColor3f(0, 0, 1.0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 100);
    glEnd();

    glPointSize(4.0f);

    glm::dvec3 pos = gz_quad->vehicle_pose().pos;
    glm::dquat rot = gz_quad->vehicle_pose().rot;

    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glRotatef(-90, 1.0f, 0.0f, 0.0f); // fix coordinate system
    auto angleAxis = glm::axis(rot);
    glRotatef(glm::angle(rot), angleAxis.x, angleAxis.y, angleAxis.z);

    glBegin(GL_POINTS);
        // Draw vehicle pos
        glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        // Draw detected obstacles
        glColor3f(1, 0, 0);
        for (auto& o : obstacles) {
             glVertex3f(o.center.x, o.center.y, o.center.z);
        }
    glEnd();
    glPopMatrix();

    glutSwapBuffers();
}

void detect()
{
    obstacles = detector->detect();
    glutPostRedisplay();
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Coav Visual Debugger");
    glutMouseFunc(updateMouse);
    glutMotionFunc(mouseMove);
    glutIdleFunc(detect);
    glutDisplayFunc(updateDisplay);
    glutMainLoop();
    return 0;
}
