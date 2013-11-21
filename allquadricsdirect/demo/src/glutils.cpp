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



bool keyHit(unsigned int key) // returns true when it sees a key change state to down
{
    static vector<bool> heldKeys;
    if (glfwGetWindowParam(GLFW_ACTIVE) == 0)
        return false;

    if (key >= heldKeys.size()) // expand to hold all keys
    {
        heldKeys.resize(key+1);
    }

    bool down = (glfwGetKey(key) != 0);
    if (down && !heldKeys[key])
    {
        heldKeys[key] = true;
        return true;
    }
    else if (!down)
    {
        heldKeys[key] = false;
    }
    return false;
}
