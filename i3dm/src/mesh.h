#ifndef MESH_H
#define MESH_H

#include <vector>

#include "algebra3.h"

namespace i3dm {

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
    
    // use the tags to filter what gets fit
    // if triangleTag.size()!=triangles.size(), we fit everything
    // otherwise, we only fit the triangles that have a tag matching activeTag
    std::vector<int> triangleTags;
    int activeTag; 

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

};

} // namespace i3dm

#endif

