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

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <coav/coav.hh>

#include "coav-control.hh"

using namespace std;

struct VisualData vdata;

void reshape_handler(GLint newWidth, GLint newHeight)
{
    vdata.window.width = newWidth;
    vdata.window.height = newHeight;

    vdata.env->set_viewport(0, 0, vdata.window.width, vdata.window.height);
    vdata.depth->set_viewport(0, 0, vdata.window.width * 0.3, vdata.window.height * 0.3);
}

void keyboard_handler( unsigned char key, int x, int y )
{
  switch ( key )
  {
    case 27: // Escape key
        glutDestroyWindow(vdata.window.ref);
        exit (0);
        break;
  }
}

void mouse_move_handler(int x, int y)
{
    vdata.env->on_mouse_move(x, y);
}

void mouse_button_handler(int button, int state, int x, int y)
{
    vdata.env->on_mouse_button(button, state, x, y);
}

void coav_loop()
{
    vdata.coav.depth_data = vdata.coav.sensor->read();
    vdata.coav.obstacles = vdata.coav.detector->detect(vdata.coav.depth_data);
    vdata.coav.avoidance->avoid(vdata.coav.obstacles);

    glutPostRedisplay();
}

void update_display()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    vdata.env->visualize(vdata.coav.vehicle, vdata.coav.obstacles);
    vdata.depth->visualize(vdata.coav.depth_data, vdata.coav.obstacles);

    glutSwapBuffers();
}

void visual_mainlopp(int argc, char* argv[], shared_ptr<MavQuadCopter> vehicle,
        shared_ptr<DepthCamera> sensor, shared_ptr<Detector> detector,
        shared_ptr<CollisionAvoidanceStrategy<MavQuadCopter>> avoidance)
{
    vdata.coav.vehicle = vehicle;
    vdata.coav.sensor = sensor;
    vdata.coav.detector = detector;
    vdata.coav.avoidance = avoidance;

    vdata.window.width = 1280;
    vdata.window.height = 960;

    vdata.env = make_shared<VisualEnvironment>(0, 0, vdata.window.width, vdata.window.height);
    vdata.depth = make_shared<VisualDepth>(0, 0, vdata.window.width * 0.3, vdata.window.height * 0.3);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(vdata.window.width, vdata.window.height);
    glutInitWindowPosition(100, 100);

    vdata.window.ref = glutCreateWindow("Coav Visual Debugger");

    glutReshapeFunc(reshape_handler);
    glutKeyboardFunc(keyboard_handler);
    glutMouseFunc(mouse_button_handler);
    glutMotionFunc(mouse_move_handler);

    glutIdleFunc(coav_loop);

    glutDisplayFunc(update_display);

    glutMainLoop();
}

#endif
