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

// A simple helper function to load a mat4 into opengl
inline void applyMat4(mat4 &m) {
	double glmat[16];
	int idx = 0;
	for (int j = 0; j < 4; j++) 
		for (int i = 0; i < 4; i++)
			glmat[idx++] = m[i][j];
	glMultMatrixd(glmat);
}

bool keyHit(unsigned int key);

#endif


