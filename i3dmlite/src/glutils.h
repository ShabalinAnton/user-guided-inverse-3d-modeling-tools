// includes for GL, GLU, GLFW, etc

#ifndef GLINC_H_
#define GLINC_H_

#include "algebra3.h"


#include <GL/glfw.h>

#ifdef __APPLE__
//#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
//#include <GL/glut.h>
#include <GL/glu.h>
#endif

void getRayFromMouse(vec2 mouse, vec3 &start, vec3 &dir);
vec2 screenToGLCoords(vec2 p);

// A simple helper function to load a mat4 into opengl
inline void applyMat4(mat4 &m) {
	double glmat[16];
	int idx = 0;
	for (int j = 0; j < 4; j++) 
		for (int i = 0; i < 4; i++)
			glmat[idx++] = m[i][j];
	glMultMatrixd(glmat);
}

// just sets some fairly arbitrary lighting
inline void setDefaultLights() {
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .4f, .3f, .3f };
	   float pos[4] = { 0, 2, 0, 0 };
       
       glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT1, GL_POSITION, pos);
       glEnable(GL_LIGHT1);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .1f, .2f, .2f};
       float pos[4] = { 0, 0, -2, 0 };
       glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT2, GL_POSITION, pos);
       glEnable(GL_LIGHT2);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .1f, .2f, .1f};
       float pos[4] = { -1, 0, 0, 0 };
       glLightfv(GL_LIGHT3, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT3, GL_POSITION, pos);
       glEnable(GL_LIGHT3);
    }
}

#endif


