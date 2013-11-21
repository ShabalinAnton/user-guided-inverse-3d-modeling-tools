#include "glutils.h"

enum { CAM_MODEL, CAM_FPS, CAM_MAX };

// Stores view info
struct Viewport {
    Viewport(): mousePos(0.0,0.0) { orientation = identity3D(); zoom = 1; mode = CAM_MODEL; pitchAngle = 0;};
	int w, h; // width and height
	vec2 mousePos;
	int mode;

	// model mode vars:
    double zoom;
    mat4 orientation;

	// first person mode vars:
    vec3 fwd;
	double pitchAngle;
    vec3 camPos;

    inline void resetCam() {
        camPos = vec3(0,0,-1);
        fwd = vec3(0,1,0);
		pitchAngle = 0;
    }

	inline vec3 computeUp() {
		vec3 sky = camPos;
		vec3 right = sky%fwd;
        if (pitchAngle > 80) pitchAngle = 80; if (pitchAngle < -80) pitchAngle = -80;
		mat4 rot = rotation3D(right, pitchAngle);
		return rot*sky;
	}

	inline void loadView() {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		if (mode == CAM_MODEL) {
			glTranslatef(0,0,-1);
			applyMat4(orientation);
			glScaled(zoom, zoom, zoom);
		} else if (mode == CAM_FPS) {
			vec3 up = computeUp();
			fwd = (up%fwd)%up;
            fwd.normalize();
			vec3 c = camPos+fwd;

			gluLookAt(camPos[0], camPos[1], camPos[2], c[0], c[1], c[2], up[0], up[1], up[2]);
		}
	}
};