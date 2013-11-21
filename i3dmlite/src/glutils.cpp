#include "glutils.h"
#include <vector>

using namespace std;

void getRayFromMouse(vec2 mouse, vec3 &start, vec3 &dir) {
	int viewport[4];
	double modelview[16], projection[16];

	glGetIntegerv(GL_VIEWPORT, viewport);			// Retrieves The Viewport Values (X, Y, Width, Height)
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);		// Retrieve The Modelview Matrix
	glGetDoublev(GL_PROJECTION_MATRIX, projection);		// Retrieve The Projection Matrix

	int mx = (int)mouse[0];
	int my = (int)mouse[1];
	my = viewport[3] - my; // convert to opengl's coordinates (y axis from bottom left)
	
	double sx,sy,sz;
	gluUnProject((float)mx, (float)my, 0, modelview, projection, viewport, &sx, &sy, &sz);
	double ex,ey,ez;
	gluUnProject((float)mx, (float)my, .3, modelview, projection, viewport, &ex, &ey, &ez);
	start = vec3(sx,sy,sz);
	vec3 end(ex, ey, ez);
	dir = end-start;
	dir.normalize();
}


vec2 screenToGLCoords(vec2 p) {
    int viewport[4];
    double modelview[16], projection[16];

    glGetIntegerv(GL_VIEWPORT, viewport);               // Retrieves The Viewport Values (X, Y, Width, Height)
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);       // Retrieve The Modelview Matrix
    glGetDoublev(GL_PROJECTION_MATRIX, projection);     // Retrieve The Projection Matrix

    int mx = (int)p[0];
    int my = (int)p[1];
    my = viewport[3] - my; // convert to opengl's coordinates (y axis from bottom left)
    
    double sx,sy,sz;
    gluUnProject((float)mx, (float)my, 0, modelview, projection, viewport, &sx, &sy, &sz);
    
    return vec2(sx, sy);
}


