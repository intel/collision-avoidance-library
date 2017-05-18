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
#include <glm/glm.hpp>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <coav/coav.hh>

auto gzrs_camera = std::make_shared<GazeboRealSenseCamera>();
auto detector = std::make_shared<DepthImageObstacleDetector>(gzrs_camera);
auto gz_quad = std::make_shared<MavQuadCopter>();
std::vector<Obstacle> obstacles;

static int window;
static int windowWidth = 1280;
static int windowHeight = 720;
static double eyeX = 40;
static double eyeY = -30;
static double eyeZ = 15;
static double zoom = 20;

static int enterX = 0, enterY = 0;

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
        glColor3f(0.9f, 0.9f, 0.9f);
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
        glColor3f(1.0, 0.4, 0.4);
        glVertex3f(-100, 0, 0);
        glVertex3f( 100, 0, 0);
        // Y axis
        glColor3f(0.4, 1.0, 0.4);
        glVertex3f(0, -100, 0);
        glVertex3f(0,  100, 0);
        // Z axis
        glColor3f(0.4, 0.4, 1.0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 100);
    glEnd();

    glTranslatef(0.0, 0.0, -0.01);

	glBegin(GL_QUADS);
	glNormal3f(0.0f, 0.0f, 1.0f);
    glColor3f(0.7f, 0.7f, 0.7f);
	glVertex3f(100.0f, 100.0f, 0.0f);
	glVertex3f(100.0f, -100.0f, 0.0f);
	glVertex3f(-100.0f, -100.0f, 0.0f);
	glVertex3f(-100.0f, 100.0f, 0.0f);
    glEnd();

    glPopMatrix();
}

void updateDisplay()
{
    glClearColor(0.8, 0.8, 0.8, 1.0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, float(windowWidth) / float(windowHeight), 1, 1000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gluLookAt(eyeX, eyeY, eyeZ, 0, 0, 0, 0, 0, 1);

    draw_grid();

    glm::dvec3 pos = gz_quad->vehicle_pose().pos;
    glm::dquat rot = gz_quad->vehicle_pose().rot;

    glPushMatrix();

    glTranslatef(pos.x, pos.y, pos.z);
    auto angleAxis = glm::axis(rot);
    glRotatef(glm::angle(rot), angleAxis.x, angleAxis.y, angleAxis.z);

    draw_vehicle();

    for (auto& o : obstacles) {
        float ox = o.center.x * cos(o.center.y);
        float oy = o.center.x * sin(o.center.y);
        float oz = o.center.x * cos(o.center.z);

        draw_obstacle(ox, oy, oz);
    }

    glPopMatrix();

    glutSwapBuffers();
}

void detect()
{
    obstacles = detector->detect();
    glutPostRedisplay();
}

void mouseMove(int mouseX, int mouseY)
{
    double angle = 3.14 * float(enterX + mouseX) / windowWidth;
    eyeX = zoom * sin(angle);
    eyeY = zoom * cos(angle);
    eyeZ = (enterY + mouseY) / 10 ;

    std::cout.precision(4);

    glutPostRedisplay();
}

void updateMouse(int button, int state, int mouseX, int mouseY)
{
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            enterX = enterX - mouseX;
            enterY = enterY - mouseY;
        } else if (state == GLUT_UP) {
            enterX = enterX + mouseX;
            enterY = enterY + mouseY;
        }
    } else if (button == 3) {
        if (state == GLUT_UP) return;
        zoom -= 5;
        eyeX = zoom / (zoom + 5) * eyeX;
        eyeY = zoom / (zoom + 5) * eyeY;
    } else if (button == 4) {
        if (state == GLUT_UP) return;
        zoom += 5;
        eyeX = zoom / (zoom - 5) * eyeX;
        eyeY = zoom / (zoom - 5) * eyeY;
    }

    glutPostRedisplay();
}

void windowReshape(GLint newWidth, GLint newHeight)
{
    windowWidth = newWidth;
    windowHeight = newHeight;

    glViewport( 0, 0, newWidth, newHeight );
}

void keyboard_handler( unsigned char key, int x, int y )
{
  switch ( key )
  {
    case 27: // Escape key
        glutDestroyWindow(window);
        exit (0);
        break;
  }
}

void init_gl()
{
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat position[] = { 30.0f, 30.0f, 30.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glShadeModel(GL_SMOOTH);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(100, 100);
    window = glutCreateWindow("Coav Visual Debugger");

    init_gl();

    glutReshapeFunc(windowReshape);
    glutMouseFunc(updateMouse);
    glutMotionFunc(mouseMove);
    glutKeyboardFunc(keyboard_handler);
    glutIdleFunc(detect);
    glutDisplayFunc(updateDisplay);
    glutMainLoop();
    return 0;
}
