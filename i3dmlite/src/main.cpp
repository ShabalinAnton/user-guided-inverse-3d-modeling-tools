// work in progress to demonstrate inverse 3D modeling modules
// this is a re-implementation of systems from my thesis, 
// with the goal of more readable and usable system than my previous research code.

// pass a mesh to fit on the command line (./i3dm "meshname")
// and then use the ui to fit primitives and use those primitives to edit the shape

#include "editorstate.h"

#include <AntTweakBar.h>

#include <fstream>
#include <map>


using namespace std;
using namespace i3dm;


EditorState g_state;


void display() {

    // setup gl state
    glClearColor(.5f,.7f,1,1);
    glEnable(GL_NORMALIZE);
    glDisable(GL_CULL_FACE);

    // clear the screen
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    // setup the camera
    g_state.set3D();

	glColor3d(1,1,1);
	g_state.mesh.draw();

    /*
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(.7,.3,.1,.3);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	quadric.draw();
	glCullFace(GL_BACK);
	quadric.draw();
    */

    g_state.curSel.draw();

    if (g_state.currentFit) {
        g_state.currentFit->draw(g_state);
    }

	

	// 2d display; for things like the cross section, etc
}


void GLFWCALL reshape( int w, int h ) {
	g_state.viewport.w = w;
	g_state.viewport.h = h;

	TwWindowSize(w, h);
}

void GLFWCALL OnMousePos(int mouseX, int mouseY) {
	g_state.viewport.mousePos = vec2(mouseX, mouseY);
	if( !TwEventMousePosGLFW(mouseX, mouseY) ) {
        g_state.viewport.mouseOnUI = false;
		if (g_state.viewport.mouseDown[0]) {
			bool hit = g_state.cast(g_state.curSel.strokeNeedsStart);
            if (hit && g_state.curSel.strokeNeedsStart) {
                g_state.curSel.strokeNeedsStart = false;
            }
		}
    } else {
        g_state.viewport.mouseOnUI = true;
    }
}

void GLFWCALL OnMouseClick( int button, int action ) {
    if (button <= 2) { // track the state of the first three mouse buttons
		g_state.viewport.mouseDown[button] = action;
	}
	if (!TwEventMouseButtonGLFW(button, action)) {
		if (button == 0 && action) { // left click
            g_state.curSel.strokeNeedsStart = !g_state.cast(true);
		}
    }
}

void TW_CALL FitFn(void *clientData) {
    if (g_state.fittingMode == FIT_STATIONARY_SWEEP) {
        delete g_state.currentFit;
        StationarySweepFitter *fit = new StationarySweepFitter();
        fit->fit(g_state, g_state.mesh, g_state.curSel);
        g_state.currentFit = fit;

    }
}

void TW_CALL ClearStrokeFn(void *notused) {
    g_state.curSel.clear();
}



int main( int argc, char **argv )
{
	
	TriangleMesh quadricMesh;

	const char *defaultInput = "example_data/default.obj";
	const char *meshfile = argc > 1 ? argv[1] : NULL;
	if (!meshfile) {
		cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
		meshfile = defaultInput;
	}
	if (!g_state.mesh.loadObj(meshfile)) {
        cerr << "Couldn't load file " << meshfile << endl;
        return 1;
    }
	if (g_state.mesh.triangles.empty()) {
		cerr << "Input mesh has no triangles -- nothing to fit!" << endl;
		return 1;
	}

	// Always recenter and scale your data before fitting!
	g_state.mesh.centerAndScale(1);
    g_state.mesh.initTags();

	
    glfwInit();

    // default window size:
    int W = 800, H = 500;
    // Open window
    glfwOpenWindowHint( GLFW_FSAA_SAMPLES, 4);
    int ok = glfwOpenWindow(W, H, 8, 8, 8, 8, 24, 8, GLFW_WINDOW);
    if( !ok ) { glfwTerminate(); return 0; }
    // setup gl window/perspective based on window height
    reshape(W,H);
	glfwSetWindowSizeCallback(reshape);

	TwInit(TW_OPENGL, NULL);
	TwWindowSize(W, H);
	TwBar *bar;
	bar = TwNewBar("Inverse3DModelingControls");

    // setup to define the UI for fitting and editing, using an ant tweak bar

    TwDefine(" GLOBAL help='Global setting bar.' "); // Message added to the help bar.
    //TwDefine(" TweakBar size='200 400' color='96 216 224' "); // change default tweak bar size and color
    // Add 'g_Zoom' to 'bar': this is a modifable (RW) variable of type TW_TYPE_FLOAT. Its key shortcuts are [z] and [Z].
    TwAddVarRW(bar, "Zoom", TW_TYPE_DOUBLE, &g_state.viewport.zoom, 
               " min=0.01 max=2.5 step=0.01 keyIncr=z keyDecr=Z help='Scale the object (1=original size).' ");
    // Add 'g_Rotation' to 'bar': this is a variable of type TW_TYPE_QUAT4F which defines the object's orientation
    TwAddVarRW(bar, "ObjRotation", TW_TYPE_QUAT4F, &g_state.viewport.antQuat, 
               " label='Object rotation' open help='Change the object orientation.' ");
    TwAddButton(bar, "ClearStrokes", ClearStrokeFn, NULL, " label='Clear Strokes' ");

    TwEnumVal rmbModes[] = 
    { 
        { 0, "Disables" },
        { 1, "Enables" } 
    };
    TwType rmbType = TwDefineEnum( "rmbModes", rmbModes, 2 );
    TwAddVarRW(bar, "rmbMode", rmbType, &g_state.rmbMode, " label='RMB Mode' ");
    
    TwEnumVal fitModeTypes[] = { { FIT_STATIONARY_SWEEP, "Stationary Sweep"}, 
                               //{ FIT_PROGRESSIVE_SWEEP,  "Progressive Sweep" }, 
                               //{ FIT_QUADRIC, "Quadric Surface" } 
    };
    TwType fitType = TwDefineEnum( "Fitting Mode", fitModeTypes, FIT_NUM_MODES );
    TwAddVarRW(bar, "Fitting Mode", fitType, &g_state.fittingMode, 
               " keyIncr=Tab keyDecr=SHIFT+Tab help='Change the fitting mode.' ");
    TwAddButton(bar, "Fit", FitFn, NULL, " label='Fit Primitive' ");
    TwAddVarRW(bar, "NoHelicalStatSweeps", TW_TYPE_BOOLCPP, &g_state.helicalMotion, " label='Helical Motions' ");
    TwAddVarRW(bar, "ShowSweepPaths", TW_TYPE_BOOLCPP, &g_state.showSweepPaths, " label='Show Sweep Paths' ");
    g_state.fittingIterations = 4;
    TwAddVarRW(bar, "FittingIters", TW_TYPE_INT32, &g_state.fittingIterations, " min=0 max=10 step=1 label='Fit Iterations' ");
    g_state.selFilterSize = 1;
    TwAddVarRW(bar, "FilterSize", TW_TYPE_INT32, &g_state.selFilterSize, " min=0 max=5 step=1 label='Segment Filter Size' ");
    g_state.editRadius2D = .2;
    TwAddVarRW(bar, "EditRad2d", TW_TYPE_DOUBLE, &g_state.editRadius2D, " min=0 max=1 step=.01 label='Edit Radius (2D)' ");
    

	// after GLFW initialization
	// directly redirect GLFW events to AntTweakBar
	glfwSetMouseButtonCallback(OnMouseClick);
	//glfwSetMousePosCallback((GLFWmouseposfun)TwEventMousePosGLFW);
	glfwSetMousePosCallback(OnMousePos);

	glfwSetMouseWheelCallback((GLFWmousewheelfun)TwEventMouseWheelGLFW);
	glfwSetKeyCallback((GLFWkeyfun)TwEventKeyGLFW);
	glfwSetCharCallback((GLFWcharfun)TwEventCharGLFW);
	

    // Set window title
    glfwSetWindowTitle( "i3dm" );

    // Enable sticky keys
    glfwEnable( GLFW_STICKY_KEYS );
	



    setDefaultLights();
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

	srand(95);



    g_state.viewport.resetCam();



    int mx, my;
    //glfwDisable( GLFW_MOUSE_CURSOR );
    glfwGetMousePos(&mx, &my);



    // Main rendering loop
    while (true) {

        if (g_state.currentFit) {
            g_state.currentFit->edit(g_state);
        }

        display();
		TwDraw();
		glfwSwapBuffers();
	
        glfwSleep( .001 ); // release control to OS for 5 ms
        

        // Check if the escape key was pressed, or if the window was closed
        if (glfwGetKey( GLFW_KEY_ESC ) || !glfwGetWindowParam( GLFW_OPENED )) {
            break;
        }
    }

    // cleanup and exit
	TwTerminate();
    glfwTerminate();
	
    return 0;
}
