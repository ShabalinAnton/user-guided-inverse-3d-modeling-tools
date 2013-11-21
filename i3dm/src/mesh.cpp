#include "mesh.h"


#include <fstream>
#include <string>
#include <sstream>



namespace {
    int getNValues(stringstream &ss, vector<int> &values, char delim = '/') {
	    values.clear();
	    string sblock;
	    if (ss >> sblock) {
		    stringstream block(sblock);
		    string s;
		    int value;
		    while (getline(block, s, delim)) {
			    stringstream valuestream(s);
			    if (valuestream >> value)
				    values.push_back(value);
                else
                    values.push_back(-1);
		    }
	    }
	    return (int)values.size();
    }
}

namespace i3dm {


bool TriangleMesh::loadObj(const char *file) {
	vertices.clear(); triangles.clear(); triangleNormals.clear();

	ifstream f(file);
	if (!f) {
		cerr << "Couldn't open file: " << file << endl;
		return false;
	}
	string line;
	vector<int> first, second, third;
	vector<int> orig;
	while (getline(f,line)) {
		if (line.empty())
			continue;
		stringstream ss(line);
		string op;
		ss >> op;
		if (op.empty() || op[0] == '#')
			continue;
		if (op == "v") {
			vec3 v;
			ss >> v;
			vertices.push_back(v);
		}
		if (op == "f")
		{
            if (!getNValues(ss, first))
				continue;
            if (!getNValues(ss, second))
                continue;
			while (getNValues(ss, third)) {
                triangles.push_back(Tri(first[0]-1, second[0]-1, third[0]-1));
                second = third;
			}
		}
	}

	computeNormalsAndAreas();

	return true;
}

} // namespace i3dm

