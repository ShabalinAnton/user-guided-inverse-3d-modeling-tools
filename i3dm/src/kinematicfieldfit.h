// kinematic field fitting library
// by James Andrews (zaphos@gmail.com)

// available for research purposes only
// not optimized for performance
// just one part of a complete kinematic surface fitting pipeline
// report bugs and feature requests to: zaphos@gmail.com

// overview: call LinParamKinematicFieldFitter<FieldType>::fitKinematicField() on your data to fit a kinematic field
// currently your data must be standard vectors of weights, positions and normals, or loaded into a TriangleMesh


#ifndef KINEMATICFIELDFITTING_H
#define KINEMATICFIELDFITTING_H

#include "algebra3.h"

#include <vector>

namespace i3dm {
    


    
// ---- FIELD TYPES ----
// Each class in this section defines the evaluation functions for a different type of kinematic field
    
// overview of field types:
// Field types are all named "*KinFieldEval" where * will describe the type of motion the field allows, for example, RotTransScaleKinFieldEval allows fields with rotation, translation and scaling motions combined, while TransKinFieldEval will only allow translation.
// The field types we define here are:
// TransKinFieldEval
// ScaleKinFieldEval
// RotTransKinFieldEval
// RotTransScaleKinFieldEval


    // Implementation note: Read if you want to *add* new field types!
// in the below *KinFieldEval classes: 
// "FuncVecs" are the vectors defining the error function v(p) dot n
// specifically, if we dot the result of the FuncVec with the parameters m, we get that error
// "GradVecs" are the gradients of those vectors with respect to the normals
/// See paper Sec. 4; GradVecs are all we need if w_p == 0, as I rec'd for Taubin's method
// "PosnGradVecs" are the gradients with respect to the position,
/// which I add in to stabilize the iterative methods; see Fig. 7 / Sec. 4.2

    // This is a field that describes pure translational motion in some direction;
    // field v(x,y,z) = (cx,cy,cz)
    // thus: v(x,y,z) dot (nx,ny,nz) = (cx, cy, cz) dot (nx, ny, nz)
    struct TransKinFieldEval {
        static int numparams() {
            return 3;
        }
        static vec3 vel(const double *c, vec3 &p) {
            vec3 a(c[0], c[1], c[2]);
            return a;
        }
        static void evalFuncVecs(const double *p, const double *n, double *c) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            c[0] = nx; c[1] = ny; c[2] = nz;
        }
        static void evalGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[0] = 1;
            cy[1] = 1;
            cz[2] = 1;
        }
        static void evalPosnGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
        }
        
        // helpers to get properties of the field
        static vec3 getDirectionOfMotion(const double *c) {
            vec3 a(c[0], c[1], c[2]);
            return a;
        }
    };
    
    // This is a field that describes a pure scaling motion from some point;
    // field v(x,y,z) = gamma (x,y,z) + (cx,cy,cz)
    // thus: v(x,y,z) dot (nx,ny,nz) = (gamma, cx, cy, cz) dot (x*nx+y*ny+z*nz, nx, ny, nz)
    // note that this can also describe translations, which corresponds to a scaling point with coordinates moved out to infinity.
    struct ScaleKinFieldEval {
        static int numparams() {
            return 4;
        }
        static vec3 vel(const double *c, vec3 &p) {
            vec3 a(c[1], c[2], c[3]);
            double gamma = c[0];
            return a + gamma*p;
        }
        static void evalFuncVecs(const double *p, const double *n, double *c) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            c[0] = x*nx+y*ny+z*nz; c[1] = nx; c[2] = ny; c[3] = nz;
        }
        static void evalGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[0] = x; cx[1] = 1;
            cy[0] = y; cy[2] = 1;
            cz[0] = z; cz[3] = 1;
        }
        static void evalPosnGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[0] = nx;
            cy[0] = ny;
            cz[0] = nz;
        }
        
        // helpers to get properties of the field
        static vec3 getCenterOfScaling(const double *c) {
            vec3 a(c[1], c[2], c[3]);
            double gamma = c[0];
            return -a/gamma;
        }
    };
    
    // This field can describe helical motion
    // field v(x,y,z) = ((rx,ry,rz) cross (x,y,z)) + (cx,cy,cz)
    // thus: v(x,y,z) dot (nx,ny,nz) = (rx,ry,rz,cx,cy,cz) dot ((x,y,z) cross (nx,ny,nz), nx, ny, nz)
    struct RotTransKinFieldEval {
        static int numparams() {
            return 6;
        }
        static vec3 vel(const double *c, vec3 &p) {
            vec3 r(c[0], c[1], c[2]), a(c[3], c[4], c[5]);
            return r%p + a;
        }
        static void evalFuncVecs(const double *p, const double *n, double *c) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            vec3 pv(x,y,z), nv(nx,ny,nz);
            vec3 pxn = (pv % nv);
            c[0] = pxn[0]; c[1] = pxn[1]; c[2] = pxn[2];
            c[3] = nx; c[4] = ny; c[5] = nz;
        }
        static void evalGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[1] =  z; cx[2] = -y; cx[3] = 1;
            cy[0] = -z; cy[2] =  x; cy[4] = 1;
            cz[0] =  y; cz[1] = -x; cz[5] = 1;
        }
        static void evalPosnGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[1] = -nz; cx[2] = ny;
            cy[0] = nz; cy[2] = -nx;
            cz[0] = -ny; cz[1] = nx;
        }
        
        // helpers to get properties of the field
        static void getAxisOfRotation(const double *params, vec3 &direction, vec3 &pointOnAxis)
        {
            direction = vec3(params[0], params[1], params[2]);
            vec3 c(params[3], params[4], params[5]);
            
            vec3 &r = direction;
            double rlen = r.length();
            c /= rlen;
            r /= rlen;
            pointOnAxis = r%(c-(c*r)*r);
        }
        
        // helper to remove pitch from helical field parameters
        static void removePitch(double *params) {
            vec3 r(params[0], params[1], params[2]);
            vec3 c(params[3], params[4], params[5]);

            c -= ((c*r)/r.length2())*r;
            
            params[3] = c[0]; params[4] = c[1]; params[5] = c[2];
        }
    };
    
    // This field can describe spiral motions
    // field v(x,y,z) = ((rx,ry,rz) cross (x,y,z)) + (cx,cy,cz) + gamma (x,y,z) 
    // thus: v(x,y,z) dot (nx,ny,nz) = (rx,ry,rz,cx,cy,cz,gamma) dot ((x,y,z) cross (nx,ny,nz), nx, ny, nz, x*nx+y*ny+z*nz)
    struct RotTransScaleKinFieldEval {
        static int numparams() {
            return 7;
        }
        static vec3 vel(const double *c, vec3 &p) {
            vec3 r(c[0], c[1], c[2]), a(c[3], c[4], c[5]);
            double gamma = c[6];
            return r%p + a + gamma*p;
        }
        static void evalFuncVecs(const double *p, const double *n, double *c) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            vec3 pv(x,y,z), nv(nx,ny,nz);
            vec3 pxn = (pv % nv);
            c[0] = pxn[0]; c[1] = pxn[1]; c[2] = pxn[2];
            c[3] = nx; c[4] = ny; c[5] = nz;
            c[6] = x*nx+y*ny+z*nz;
        }
        static void evalGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[1] =  z; cx[2] = -y; cx[3] = 1; cx[6] = x;
            cy[0] = -z; cy[2] =  x; cy[4] = 1; cy[6] = y;
            cz[0] =  y; cz[1] = -x; cz[5] = 1; cz[6] = z;
        }
        static void evalPosnGradVecs(const double *p, const double *n, double *cx, double *cy, double *cz) {
            double x = p[0], y = p[1], z = p[2];
            double nx = n[0], ny = n[1], nz = n[2];
            for (int i = 0; i < numparams(); i++) {
                cx[i] = cy[i] = cz[i] = 0;
            }
            cx[1] = -nz; cx[2] = ny; cx[6] = nx;
            cy[0] = nz; cy[2] = -nx; cy[6] = ny;
            cz[0] = -ny; cz[1] = nx; cz[6] = nz;
        }
        
        // helpers to get properties of the field
        // helpers to get properties of the field
        static void getAxisOfRotation(const double *params, vec3 &direction, vec3 &pointOnAxis)
        {
            direction = vec3(params[0], params[1], params[2]);
            vec3 c(params[3], params[4], params[5]);
            
            vec3 &r = direction; // for brevity
            double rlen = r.length();
            c /= rlen;
            r /= rlen;
            double gamma = params[6] / rlen;
            
            pointOnAxis = r % ( (c-(c*r)*r+(r%c)*gamma)/(1+gamma*gamma) );
        }
    };
// ---- END OF FIELD TYPES ----
    
    
    struct TriangleMesh; // very simple triangle mesh class, defined below
    
  
    // helper function for the below class (exposed because ... templates)
    void getParamsHelper(vector<double> &tofill, vector<double> &M, vector<double> &N, int numparams, bool recomputeErrors = true);
                           
    
// This class provides the functions to solve for the parameters of a kinematic field (of type specified by LinFieldEval)
    template<typename LinFieldEval>
    struct LinParamKinematicFieldFitter {
        vector<double>  c, cx, cy, cz, cxp, cyp, czp;
        vector<double> M, N;
        int curSize;
        LinParamKinematicFieldFitter() { clear(); }
        void clear() {
            const int n = LinFieldEval::numparams();
            M.clear(); N.clear(); M.resize(n*n,0.0); N.resize(n*n,0.0);
            c.resize(n); cx.resize(n); cy.resize(n); cz.resize(n);
            cxp.resize(n); cyp.resize(n); czp.resize(n);
            curSize = n;
        }
        
        // w_p is set to a default of zero, so error is only measured in the surface normals
        void addElement(double scale, const vec3 &p, const vec3 &n) {
            const int numpar = LinFieldEval::numparams();
            LinFieldEval::evalFuncVecs(p.n, n.n, &c[0]);
            LinFieldEval::evalGradVecs(p.n, n.n, &cx[0], &cy[0], &cz[0]);
            
            // add cross products to accumulator matrices
            for (int i = 0; i < numpar; i++) {
                for (int j = 0; j < numpar; j++) {
                    M[i*numpar+j] += c[i]*c[j] * scale;
                    N[i*numpar+j] += (cx[i]*cx[j] + cy[i]*cy[j] + cz[i]*cz[j]) * scale;
                }
            }
        }

        // w_p specifies how much error is measured in the data positions as well
        void addElement(double scale, const vec3 &p, const vec3 &n, double w_p) {
            const int numpar = LinFieldEval::numparams();
            
            LinFieldEval::evalFuncVecs(p.n, n.n, &c[0]);
            LinFieldEval::evalGradVecs(p.n, n.n, &cx[0], &cy[0], &cz[0]);
            LinFieldEval::evalPosnGradVecs(p.n, n.n, &cxp[0], &cyp[0], &czp[0]);
            
            // add cross products to accumulator matrices
            for (int i = 0; i < numpar; i++) {
                for (int j = 0; j < numpar; j++) {
                    M[i*numpar+j] += c[i]*c[j] * scale;
                    N[i*numpar+j] += (cx[i]*cx[j] + cy[i]*cy[j] + cz[i]*cz[j]) * scale;
                    N[i*numpar+j] += w_p * (cxp[i]*cxp[j] + cyp[i]*cyp[j] + czp[i]*czp[j]) * scale;
                }
            }
        }

        // w_p specifies how much error is measured in the data positions as well
        // this adds HEIV-type rescaling; use this function for iterative refinement of an initial result
        void addElement_rescaledByHEIV(vector<double> &lastIterParams, double scale, const vec3 &p, const vec3 &n, double w_p = .001) {
            const int numpar = LinFieldEval::numparams();
            
            LinFieldEval::evalFuncVecs(p.n, n.n, &c[0]);
            LinFieldEval::evalGradVecs(p.n, n.n, &cx[0], &cy[0], &cz[0]);
            LinFieldEval::evalPosnGradVecs(p.n, n.n, &cxp[0], &cyp[0], &czp[0]);

            // HEIV-type rescaling:
            double eval = 0; vec3 dp(0), dn(0);
            vector<double> &l = lastIterParams; // alias for brevity
            for (int i = 0; i < numpar; i++) {
                eval += l[i]*c[i];
                dn[0] += l[i]*cx[i]; dn[1] += l[i]*cy[i]; dn[2] += l[i]*cz[i];
                dp[0] += l[i]*cxp[i]; dp[1] += l[i]*cyp[i]; dp[2] += l[i]*czp[i];
            }
            double evalSq = eval*eval;
            // as long as w_p is non-zero and the lastIterParams are non-zero this should generally be not divide by zero:
            double invGradSq = 1.0 / (dn*dn + w_p*(dp*dp));
            
            double Mscale = invGradSq;
            double Nscale = evalSq * invGradSq * invGradSq;
            
            // add cross products to accumulator matrices
            for (int i = 0; i < numpar; i++) {
                for (int j = 0; j < numpar; j++) {
                    M[i*numpar+j] += c[i]*c[j] * scale * Mscale;
                    N[i*numpar+j] += (cx[i]*cx[j] + cy[i]*cy[j] + cz[i]*cz[j]) * scale * Nscale;
                    N[i*numpar+j] += w_p * (cxp[i]*cxp[j] + cyp[i]*cyp[j] + czp[i]*czp[j]) * scale * Nscale;
                }
            }
        }
        
        inline void getParams(vector<double> &tofill)
        {
            getParamsHelper(tofill, M, N, LinFieldEval::numparams());
        }

        // fits a kinematic field of type LinFieldEval to the given points and normals, with a 'weight' specified for each point indicating its importance
        void fitKinematicField(const std::vector<double> &weights, const std::vector<vec3> &positions, const std::vector<vec3> &normals, std::vector<double> &parametersOut, double w_p = 0)
        {
            clear();
            size_t num = weights.size();
            
            if (w_p == 0) { 
                for (size_t i = 0; i < num; i++) { addElement(weights[i], positions[i], normals[i]); }
            } else {
                for (size_t i = 0; i < num; i++) { addElement(weights[i], positions[i], normals[i], w_p); }
            }
            
            getParams(parametersOut);
        }

        // fits a kinematic field of type LinFieldEval to the given points and normals, with a 'weight' specified for each point indicating its importance
        void fitKinematicField_HEIV(const std::vector<double> &weights, const std::vector<vec3> &positions, const std::vector<vec3> &normals, std::vector<double> &parametersOut, int iters = 4, double w_p = .001)
        {
            
            size_t num = weights.size();

            clear();
            
            // we use the Taubin-style method with no initial guess first, to find our initial guess
            for (size_t i = 0; i < num; i++) { addElement(weights[i], positions[i], normals[i], w_p); }
            getParams(parametersOut);

            // we iteratively update our estimate, reweighting based on the last estimate and then refitting
            for (int iter = 0; iter < iters; iter++) {
                clear();
                for (size_t i = 0; i < num; i++) { addElement_rescaledByHEIV(parametersOut, weights[i], positions[i], normals[i], w_p); }
                getParams(parametersOut);
            }
            
            
        }
        
        // same as above, but without weights
        void fitKinematicField(const std::vector<vec3> &positions, const std::vector<vec3> &normals, std::vector<double> &parametersOut, double w_p = 0)
        {
            clear();
            size_t num = positions.size();
            
            if (w_p == 0) {
                for (size_t i = 0; i < num; i++) { addElement(1.0, positions[i], normals[i]); }
            } else {
                for (size_t i = 0; i < num; i++) { addElement(1.0, positions[i], normals[i], w_p); }
            }
            
            getParams(parametersOut);
        }
        
        // simple helper to fit triangle meshes; just calls the above function
        inline void fitKinematicField(TriangleMesh &mesh, std::vector<double> &parameter);
        inline void fitKinematicField_HEIV(TriangleMesh &mesh, std::vector<double> &parameter, int iters = 4, double w_p = .001);
        
    };
    
    
    
    
    // very simple triangle mesh class so that we can load and work with triangle meshes
    
    // stores triangles
    struct Tri {
        int ind[3];
        Tri(int a, int b, int c) { ind[0] = a; ind[1] = b; ind[2] = c; }
        Tri() { ind[0] = ind[1] = ind[2] = 0; }
    };
    
    // stores triangle meshes
    struct TriangleMesh {
        
        std::vector<Tri> triangles;
        std::vector<vec3> triangleNormals;
        std::vector<double> triangleAreas;
        
        std::vector<vec3> vertices;
        std::vector<vec3> vertexNormals;
        std::vector<double> vertexAreas;
        
        void computeNormalsAndAreas() {
            triangleNormals.resize(triangles.size()); triangleAreas.resize(triangles.size());
            for (size_t i = 0; i < triangles.size(); i++) {
                vec3 edge1 = vertices[triangles[i].ind[1]] - vertices[triangles[i].ind[0]];
                vec3 edge2 = vertices[triangles[i].ind[2]] - vertices[triangles[i].ind[1]];
                vec3 n = edge1 % edge2;
                double len = n.length();
                if (fabs(len) > 0) n /= len;
                triangleNormals[i] = n;
                triangleAreas[i] = len / 2;
            }
            
            vertexNormals.resize(vertices.size()); vertexAreas.resize(vertices.size());
            for (size_t i = 0; i < vertexNormals.size(); i++) {
                vertexNormals[i] = vec3(0);
                vertexAreas[i] = 0;
            }
            for (size_t i = 0; i < triangles.size(); i++) {
                
                for (int ii = 0; ii < 3; ii++) {
                    int ind = triangles[i].ind[ii];
                    vertexNormals[ind] += triangleNormals[i];
                    vertexAreas[ind] += triangleAreas[i];
                }
            }
            for (size_t i = 0; i < vertexNormals.size(); i++) {
                vertexNormals[i].normalize();
            }
        }
        void clear() {
            triangles.clear(); triangleNormals.clear(); vertices.clear();
        }
        bool loadObj(const char *file);
        
        void centerAndScale(double scale) {
            if (vertices.empty())
                return;
            
            vec3 maxp = vertices[0], minp = vertices[0];
            for (vector<vec3>::iterator it = vertices.begin(); it != vertices.end(); ++it) {
                maxp = max(*it, maxp); // max and min def'd in algebra3.h take the max or min componentwise from each vector
                minp = min(*it, minp);
            }
            vec3 center = (maxp+minp)*.5;
            vec3 size = maxp-minp;
            double maxSizeInv = MAX(size[0],MAX(size[1],size[2]));
            if (maxSizeInv == 0) // mesh is just one point
                return;
            maxSizeInv = 1.0/maxSizeInv;
            for (vector<vec3>::iterator it = vertices.begin(); it != vertices.end(); ++it) {
                *it = (*it-center)*maxSizeInv*scale;
            }
        }
        
    };

    // The fitting function for meshes (just calls the more general fitting function, after ensure the needed mesh normals and vertex areas are computed)
    template<typename LinFieldEval>
    void LinParamKinematicFieldFitter<LinFieldEval>::fitKinematicField(TriangleMesh &mesh, std::vector<double> &parametersOut)
    {
        if (mesh.vertices.size() != mesh.vertexNormals.size())
        {
            mesh.computeNormalsAndAreas();
        }
    
        fitKinematicField(mesh.vertexAreas, mesh.vertices, mesh.vertexNormals, parametersOut);
    }

    template<typename LinFieldEval>
    void LinParamKinematicFieldFitter<LinFieldEval>::fitKinematicField_HEIV(TriangleMesh &mesh, std::vector<double> &parametersOut, int iters, double w_p)
    {
        if (mesh.vertices.size() != mesh.vertexNormals.size())
        {
            mesh.computeNormalsAndAreas();
        }
    
        fitKinematicField_HEIV(mesh.vertexAreas, mesh.vertices, mesh.vertexNormals, parametersOut, iters, w_p);
    }

    

} // namespace i3dm


#endif




