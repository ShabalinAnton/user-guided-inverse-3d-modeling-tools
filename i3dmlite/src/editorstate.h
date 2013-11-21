#ifndef EDITOR_STATE_H
#define EDITOR_STATE_H

#include "view.h"

#include "quadricfitting.h"

#include "kffProfileEditor.h"
#include "kinematicfieldfit.h"
#include "glutils.h"


namespace i3dm {

// fitting modes
enum { FIT_STATIONARY_SWEEP = 0, FIT_NUM_MODES }; // FIT_QUADRIC, etc, to be added (hopefully in the not-too-distant future)

struct EditorState;

// abstract struct to define how to fit each primitive type, as well as how the primitive fit will be visualized and the UI to use the fit for editing
struct Fitter {
    virtual void fit(EditorState &state, TriangleMesh &mesh, SelectionStroke &stroke) = 0;
    virtual void edit(EditorState &state) = 0; // todo pass mouse state to this
    virtual void draw(EditorState &state) = 0;
};

// the full state of the editor
struct EditorState {
    int fittingMode;
    SelectionStroke curSel;
    Viewport viewport;
    TriangleMesh mesh;
    Fitter *currentFit;
    // etc.
    
    // sweep settings:
    bool helicalMotion;
    bool showSweepPaths;
    int selFilterSize;
    int fittingIterations;
    double editRadius2D;
    int rmbMode;

    EditorState() {
        currentFit = NULL;
    }

    void set3D() {
        setProj3D(viewport.w, viewport.h, 0, 1.5);
        viewport.loadView();
    }

    void set2D() {
        setProj2D(viewport.w, viewport.h, 2, 3.0f);
        // do any 2D camera movements here.
    }

    bool cast(bool startStroke = false) {
        vec3 start, dir;
        float t;
        set3D();
        getRayFromMouse(viewport.mousePos, start, dir);
        int tri = mesh.intersect(start, dir, t);
        
        if (tri >= 0) {
            curSel.add(start+dir*t, tri, startStroke);
            mesh.triangleTags[tri] = mesh.activeTag;
            return true;
        }
        return false;
    }
};

// error of fit objects are used to define how a primitive fits sample points
// the mesh class uses these to allow segmentation according to error of fit.
struct RotTransKinFieldError : ErrorOfFit {
    double *params;
    RotTransKinFieldError(double *params) : params(params) {}
    double err(vec3 p, vec3 n) {
        vec3 v = RotTransKinFieldEval::vel(params,p);
        if (v.length2() < .00001) return 0;
        v.normalize();
        return fabs(v*n);
    }
    double minErr() {
        return .01;
    }
};

// the fitting, rendering, and editing routines for the first inverse 3d modeling module: stationary sweeps
struct StationarySweepFitter : Fitter {
    // put global settings variables here:
    static int followStroke;
    
    // state from fit:
    ProfileEditor profEd;
    LinParamKinematicFieldFitter<RotTransKinFieldEval> fitter;
    std::vector<double> fitParams;
    vec3 lastMouse;

    // fitting fn
    virtual void fit(EditorState &state, TriangleMesh &mesh, SelectionStroke &stroke) {
        mesh.selectFromStroke(stroke);
        fitter.fitKinematicField(mesh, fitParams);
        for (int i = 0; i < state.fittingIterations; i++) {
            mesh.selectFromStroke(stroke);
            RotTransKinFieldError err(&fitParams[0]);
            mesh.growSelection(err);
            fitter.fitKinematicField(mesh, fitParams);
        }
        if (!state.helicalMotion) {
            RotTransKinFieldEval::removePitch(&fitParams[0]);
        }
        // dialate then erode by the number of filter steps -> perform a "closing" filter
        for (int i = 0; i < state.selFilterSize; i++) {
            mesh.filter(false);
        }
        for (int i = 0; i < state.selFilterSize; i++) {
            mesh.filter(true);
        }
        // erode then dialate for an "opening" filter
        for (int i = 0; i < state.selFilterSize; i++) {
            mesh.filter(true);
        }
        for (int i = 0; i < state.selFilterSize; i++) {
            mesh.filter(false);
        }
        ProfileEditor::Ray r;
        RotTransKinFieldEval::getAxisOfRotation(&fitParams[0], r.dir, r.start);
        profEd.load(&mesh, r, fitParams, stroke.path.empty() ? vec3(0,0,0) : stroke.path[0].p);

        lastMouse = state.viewport.mousePos;
    }

    // edit fn
    virtual void edit(EditorState &state) {
        state.set2D();
        
        if (!state.viewport.mouseOnUI) {
            if (state.viewport.mouseDown[0]) {
                double r = state.editRadius2D;
                vec2 glposm = screenToGLCoords(state.viewport.mousePos);
                vec2 glposmLast = screenToGLCoords(lastMouse);
                vec2 d = glposm-glposmLast;
                profEd.move(glposm, d, r*r, true);
                profEd.pushParents();
            }
            if (state.viewport.mouseDown[1]) { // rmb
                vec2 glposm = screenToGLCoords(state.viewport.mousePos);
                double r = state.editRadius2D;
                profEd.toggleOnOff(glposm, r*r, state.rmbMode == 1);
            }
        }

        lastMouse = state.viewport.mousePos;
    }

    virtual void draw(EditorState &state) {
        state.set2D();
        profEd.draw();

        state.set2D();
        vec2 glposm = screenToGLCoords(state.viewport.mousePos);
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_LIGHTING);
        glColor4d(1,1,0,.5);
        glBegin(GL_POLYGON);
        for (float a = 0; a < 2*M_PI; a+=.4) {
            vec2 p = glposm + state.editRadius2D * vec2(cos(a),sin(a));
            glVertex2dv(&p[0]);
        }
        glEnd();
        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
        if (state.showSweepPaths) {
            state.set3D();
            vec3 starts[3] = {vec3(0,0,0), vec3(.3,.5,.2), vec3(-.5,0,.5)};
            if (!state.curSel.path.empty()) {
                srand(0);
                for (int i = 0; i < 3; i++) {
                    int si = rand()%state.curSel.path.size();
                    starts[i] = state.curSel.path[si].p;
                }
            } else if (!state.mesh.vertices.empty()) {
                srand(0);
                for (int i = 0; i < 3; i++) {
                    int vi = rand()%state.mesh.vertices.size();
                    starts[i] = state.mesh.vertices[vi];
                }
            }
            for (int i = 0; i < 3; i++) {
                vec3 p = starts[i];
                glColor3d(1,0,0);
                glBegin(GL_LINE_STRIP);
                for (int steps = 0; steps < 10000; steps++) {
                    vec3 v = RotTransKinFieldEval::vel(&fitParams[0], p);
                    v.normalize();
                    p += v*.001;
                    glVertex3dv(&p[0]);
                }
                glEnd();
            }
        }
    }
};


} // namespace i3dm


#endif

