#ifndef MESH_H
#define MESH_H

#include <vector>
#include <queue>

#include "algebra3.h"
#include "glutils.h"

namespace i3dm {

// stores triangles
struct Tri {
	int ind[3];
    int neigh[3];
	Tri(int a, int b, int c) { ind[0] = a; ind[1] = b; ind[2] = c; }
	Tri() { ind[0] = ind[1] = ind[2] = 0; neigh[0] = neigh[1] = neigh[2] = -1; }
};

// a data point with position, normal and weight
struct data_pnw {
	vec3 p, n;
	double w;

	data_pnw(vec3 &p, vec3 &n, double w) : p(p), n(n), w(w) {}
	data_pnw() {}
};

// selections on the surface of a triangle mesh
struct SelectionStroke {
	struct StrokeEl {
		vec3 p;
		int tri;
        bool startStroke;

        StrokeEl(vec3 p, int tri, bool start = false) : p(p), tri(tri), startStroke(start) {}
	};
    vector<StrokeEl> path;
    bool strokeNeedsStart;

    void add(vec3 p, int tri, bool start = false) {
        path.push_back(StrokeEl(p, tri, start));
    }

    void clear() { path.clear(); strokeNeedsStart = true; }

    void draw() {
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glEnable(GL_POINT_SMOOTH);
        glLineWidth(10);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glColor3d(1,1,0);
        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i < path.size(); ++i) {
            if (path[i].startStroke) {
                glEnd();
                glBegin(GL_LINE_STRIP);
            }
            glVertex3dv(&path[i].p[0]);
        }
        glEnd();
        glPointSize(9);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < path.size(); ++i) {
            glVertex3dv(&path[i].p[0]);
        }
        glEnd();
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
    }
};

// abstract struct used for mesh segmentation
struct ErrorOfFit {
    virtual double err(vec3 p, vec3 n) = 0;
    virtual double minErr() = 0;
};

// stores triangle meshes
struct TriangleMesh {
	
	std::vector<Tri> triangles;
	std::vector<vec3> triangleNormals;
	std::vector<double> triangleAreas;
    
	std::vector<vec3> vertices;
    std::vector<vec3> vertexNormals;
    std::vector<double> vertexAreas;
    
    // use the tags to filter what gets fit
    // if triangleTag.size()!=triangles.size(), we fit everything
    // otherwise, we only fit the triangles that have a tag matching activeTag
    std::vector<int> triangleTags;
    int activeTag;

    std::vector<int> vertexTags;

    void clearCurrentSel(int defaultTag = -1) {
        for (size_t i = 0; i < triangleTags.size(); i++) {
            if (triangleTags[i] == activeTag)
                triangleTags[i] = defaultTag;
        }
    }
    void selectFromStroke(SelectionStroke &stroke, int defaultTag = -1) {
        clearCurrentSel(defaultTag);
        for (size_t i = 0; i < stroke.path.size(); i++) {
            triangleTags[stroke.path[i].tri] = activeTag;
        }
    }
    void growSelection(ErrorOfFit &fit, int defaultTag = -1) {
        std::queue<int> q;
        double thresh = fit.minErr();
        for (size_t i = 0; i < triangleTags.size(); i++) {
            if (triangleTags[i] == activeTag) {
                q.push((int)i);
                thresh = MAX(thresh, fit.err(centroid(i), triangleNormals[i]));
            }
        }
        thresh *= 1.1;
        while (!q.empty()) {
            int tri = q.front();
            q.pop();
            for (int ni = 0; ni < 3; ni++) {
                int ntri = triangles[tri].neigh[ni];
                if (ntri >= 0 && ntri < triangleTags.size() && triangleTags[ntri] == defaultTag) {
                    double e = fit.err(centroid(ntri), triangleNormals[ntri]);
                    if (e < thresh) {
                        triangleTags[ntri] = activeTag;
                        q.push(ntri);
                    }
                }
            }
        }
    }

    void initTags(int defaultTag = -1, int activeTag = 0) {
        triangleTags.resize(triangles.size(), defaultTag);
        vertexTags.resize(vertices.size());
        this->activeTag = activeTag;
    }
    
    void computeTriangleAdjacencies() {
        vector< vector< pair<int, pair<int,int> > > > edges(vertices.size());
        for (size_t i = 0; i < triangles.size(); i++) {
            for (int ii = 0; ii < 3; ii++) {
                int jj = (ii+1)%3;
                int v1 = triangles[i].ind[ii];
                int v2 = triangles[i].ind[jj];
                bool foundEdge = false;
                for (size_t j = 0; j < edges[v2].size(); j++) {
                    if (edges[v2][j].first == v1) {
                        int opp = edges[v2][j].second.first;
                        triangles[i].neigh[ii] = opp;
                        triangles[opp].neigh[edges[v2][j].second.second] = (int)i;
                        foundEdge = true;
                        break;
                    }
                }
                if (!foundEdge) {
                    edges[v1].push_back(pair<int,pair<int,int> >(v2, pair<int,int>(i, ii)));
                }
            }
        }
    }

	void computeNormals() {
		triangleNormals.resize(triangles.size());
		for (size_t i = 0; i < triangles.size(); i++) {
			vec3 edge1 = vertices[triangles[i].ind[1]] - vertices[triangles[i].ind[0]];
			vec3 edge2 = vertices[triangles[i].ind[2]] - vertices[triangles[i].ind[1]];
			triangleNormals[i] = edge1 % edge2;
			triangleNormals[i].normalize();
		}
	}
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
                                        
    vec3 centroid(int i) {
        if (i < 0 || i > triangles.size()) {
            return vec3(0);
        }
        vec3 c =
        (vertices[triangles[i].ind[0]] +
         vertices[triangles[i].ind[1]] +
         vertices[triangles[i].ind[2]])/3.0;
        return c;
    }

	// returns -1 on no intersection; returns the index of the intersected triangle otherwise
	// on intersection, bestT will include the 'time of intersection' (so its pos = start+dir*bestT)
	int intersect(vec3 start, vec3 dir, float &bestT) {
        int tFound = -1;
		int triInd = 0;
        for (vector<Tri>::iterator it = triangles.begin(); it != triangles.end(); ++it, ++triInd) {
            vec3 &v0 = vertices[(*it).ind[0]];
            vec3 &v1 = vertices[(*it).ind[1]];
            vec3 &v2 = vertices[(*it).ind[2]];

            vec3 amb = v0 - v1;
            float a = amb[0], b = amb[1], c = amb[2];
            vec3 amc = v0 - v2;
            float d = amc[0], e = amc[1], f = amc[2];
            //vec4 dir = ray.direction();
            float g = dir[0], h = dir[1], i = dir[2];
            vec3 ame = v0 - start;
            float j = ame[0], k = ame[1], l = ame[2];
            float M = a*(e*i-h*f)+b*(g*f-d*i)+c*(d*h-e*g);
            if (M == 0)
                continue;
            float t = -(f*(a*k-j*b)+e*(j*c-a*l)+d*(b*l-k*c))/M;
            float gamma = (i*(a*k-j*b)+h*(j*c-a*l)+g*(b*l-k*c))/M;
            if (gamma < 0 || gamma > 1)
                continue;
            float beta = (j*(e*i-h*f)+k*(g*f-d*i)+l*(d*h-e*g))/M;
            if (beta < 0 || beta > 1 - gamma)
                continue;
            //if (t < 0)
              //  continue;
            if (tFound == -1 || t < bestT) {
                tFound = triInd;
                bestT = t;
            }
        }
        return tFound;
    }

    // basic erode/dialate morphology, to be used for filtering a selection
    void filter(bool erode, int tag, int bg = -1) {
        // find the max used tag; so maxTag+1 will be unused
        int maxTag = bg;
        for (size_t i = 0; i < triangleTags.size(); i++) {
            maxTag = MAX(maxTag,triangleTags[i]);
        }
        // shrink or grow the boundary of the segmentation
        for (size_t i = 0; i < triangleTags.size(); i++) {
            if (triangleTags[i] == tag) {
                for (int ii = 0; ii < 3; ii++) {
                    int ni = triangles[i].neigh[ii];
                    if (ni >= 0 && ni < triangleTags.size()) {
                        if (triangleTags[ni] == bg) { // we're on a boundary between background and foreground
                            if (erode) {
                                triangleTags[i] = maxTag+1; // shrink the boundary of the segmentation
                            } else {
                                triangleTags[ni] = maxTag+1; // grow the boundary oif the segmentation
                            }
                        }
                    }
                }
            }
        }
        // map maxTag+1 -> foreground or background (depending on whether we're eroding or dialating)
        int newTag = tag;
        if (erode) {
            newTag = bg;
        }
        for (size_t i = 0; i < triangleTags.size(); i++) {
            if (triangleTags[i] == maxTag+1) {
                triangleTags[i] = newTag;
            }
        }
    }
    inline void filter(bool erode, int bg = -1) { // shortcut for the common case that we filter the activeTags
        filter(erode, activeTag, bg);
    }

    void draw() {
	    glBegin(GL_TRIANGLES);
	    for (size_t i = 0; i < triangles.size(); i++) {
            
            if (i < triangleTags.size()) {
                if (triangleTags[i] == activeTag) {
                    glColor3d(1,0,0);
                } else {
                    glColor3d(1,1,1);
                }
            }
		    glNormal3dv(&triangleNormals[i][0]);
		    for (int ii = 0; ii < 3; ii++) {
			    glVertex3dv(&vertices[ triangles[i].ind[ii] ][0]);
		    }
	    }
	    glEnd();
        
        // debug code to visualize adjacencies
#ifdef ENABLE_VISUALIZE_ADJACENCIES_DEBUG_CODE
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glColor3d(1,1,0);
        glLineWidth(2);
        glBegin(GL_LINES);
        for (size_t i = 0; i < triangles.size(); i++) {
            vec3 c = centroid(i);
            for (int ii = 0; ii < 3; ii++){
                if (triangles[i].neigh[ii] < 0) continue;
                vec3 cn = centroid(triangles[i].neigh[ii]);
                glVertex3dv(&c[0]);
                glVertex3dv(&cn[0]);
            }
        }
        glEnd();
#endif
    }


};



// helper function: computes the six order-4 dunavant points and weights
// a, b, c are the triangle vertex positions
// i is the index of the dunavant point (must be in range [0,5])
// out is the i'th dunavant point, wt is the corresponding dunavant weight
inline void computeIthDunavant(vec3 &a, vec3 &b, vec3 &c, int i, vec3 &out, double &wt) {
	// dunavant weights and coordinates (for polynomials of order 4)
	double w[6] = {0.223381589678011, 0.223381589678011, 0.223381589678011, 0.109951743655322, 0.109951743655322, 0.109951743655322};
	double x[6] = {0.10810301816807, 0.445948490915965, 0.445948490915965, 0.81684757298045896, 0.091576213509771007, 0.091576213509771007};
	double y[6] = {0.445948490915965, 0.445948490915965, 0.10810301816807, 0.091576213509771007, 0.091576213509771007, 0.81684757298045896};
	out = a*(1.0-x[i]-y[i]) + b*(x[i]) + c*(y[i]);
	wt = w[i];
}

// iterator to traverse dunavant points of a triangle mesh
class dunavantIterator
{
    public:
        dunavantIterator(TriangleMesh &m) : mesh(&m), triInd(0), subInd(0) { findStartTri(); computeData(); }
        dunavantIterator operator++() { increment(); return *this; }
        dunavantIterator operator++(int junk) { increment(); return *this; }
        data_pnw& operator*() { return data; }
        data_pnw* operator->() { return &data; }
		bool operator==(const dunavantIterator& rhs) { 
			return mesh == rhs.mesh && triInd == rhs.triInd && subInd == rhs.subInd; 
		}
        bool operator!=(const dunavantIterator& rhs) { 
			return mesh != rhs.mesh || triInd != rhs.triInd || subInd != rhs.subInd;  
		}
		static dunavantIterator end(TriangleMesh &m) {
			dunavantIterator it(m);
			it.triInd = (int)it.mesh->triangles.size();
			return it;
		}
    private:
        TriangleMesh *mesh;
		int triInd, subInd;
		data_pnw data;
    
        void findStartTri() {
            while (!mesh->triangleTags.empty() && mesh->triangleTags.size()==mesh->triangles.size()
                   && triInd < mesh->triangles.size()
                   && mesh->triangleTags[triInd] != mesh->activeTag) {
                triInd++;
            }
        }

		inline void increment() {
			if (done()) return;

			subInd++;
			if (subInd > 5) {
				subInd = 0;
				triInd++;
                
                //if the triangleTags vector is set up, skip over non-active triangles 
                while (!mesh->triangleTags.empty() && mesh->triangleTags.size()==mesh->triangles.size()
                       && triInd < mesh->triangles.size()
                       && mesh->triangleTags[triInd] != mesh->activeTag) {
                    triInd++;
                }
			}
			computeData();
		}
		inline bool done() {
			return triInd >= (int)mesh->triangles.size();
		}
		inline void computeData() {
			if (done()) {
				return;
			}

			computeIthDunavant(
				mesh->vertices[mesh->triangles[triInd].ind[0]], 
				mesh->vertices[mesh->triangles[triInd].ind[1]],
				mesh->vertices[mesh->triangles[triInd].ind[2]],
				subInd, data.p, data.w
				);
			data.n = mesh->triangleNormals[triInd];
		}
};	

} // namespace i3dm

#endif

