#include "glutils.h"

enum { CAM_MODEL, CAM_FPS, CAM_MAX };

inline void setProj3D(int width, int height, double column = 0, double numcolumns = 2) {
    int w = width/numcolumns;
    int h = height;
    glViewport(w*column, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, w / MAX(h, 1.0), .1, 200.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

inline void setProj2D(int width, int height, float column = 1, float numcolumns = 2) {
    int w = width/numcolumns;
    int h = height;
    glViewport(w*column, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1,1,-1,1,1,-1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

// Stores view info
struct Viewport {
    Viewport() : mousePos(0.0,0.0) { 
		zoom = .7; mode = CAM_MODEL; pitchAngle = 0;
        //antQuat[0] = 1; antQuat[1] = 0; antQuat[2] = 0; antQuat[3] = 0;
		mouseDown[0] = mouseDown[1] = mouseDown[2] = false; 
	}
	int w, h; // width and height
	vec2 mousePos;
	int mouseDown[3];
    bool mouseOnUI;
	int mode;

	// model mode vars:
    double zoom;
    float antQuat[4];

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

    // Routine to convert a quaternion to a 4x4 matrix
    // ( input: quat = float[4]  output: mat = float[4*4] )
    // based on AntTweakBar's sample code
    inline void uploadAntQuaternionToGL(const float *quat)
    {
        float mat[4*4];
        float yy2 = 2.0f * quat[1] * quat[1];
        float xy2 = 2.0f * quat[0] * quat[1];
        float xz2 = 2.0f * quat[0] * quat[2];
        float yz2 = 2.0f * quat[1] * quat[2];
        float zz2 = 2.0f * quat[2] * quat[2];
        float wz2 = 2.0f * quat[3] * quat[2];
        float wy2 = 2.0f * quat[3] * quat[1];
        float wx2 = 2.0f * quat[3] * quat[0];
        float xx2 = 2.0f * quat[0] * quat[0];
        mat[0*4+0] = - yy2 - zz2 + 1.0f;
        mat[0*4+1] = xy2 + wz2;
        mat[0*4+2] = xz2 - wy2;
        mat[0*4+3] = 0;
        mat[1*4+0] = xy2 - wz2;
        mat[1*4+1] = - xx2 - zz2 + 1.0f;
        mat[1*4+2] = yz2 + wx2;
        mat[1*4+3] = 0;
        mat[2*4+0] = xz2 + wy2;
        mat[2*4+1] = yz2 - wx2;
        mat[2*4+2] = - xx2 - yy2 + 1.0f;
        mat[2*4+3] = 0;
        mat[3*4+0] = mat[3*4+1] = mat[3*4+2] = 0;
        mat[3*4+3] = 1;

        glMultMatrixf(mat);
    }

	inline void loadView() {
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		if (mode == CAM_MODEL) {
			glTranslatef(0,0,-1);
            uploadAntQuaternionToGL(antQuat);
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