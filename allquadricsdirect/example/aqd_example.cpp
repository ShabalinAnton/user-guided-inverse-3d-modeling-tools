// demonstrates usage of the quadric fitting library
// pass a mesh to fit on the command line (./aqd_example meshname)
// and this will fit all quadric types to the *whole* input mesh (no segmentation is performed)
// and then output the results to the console

// note that the order of quadric parameters is:
//  constant, x coef., y coef., z coef., x*x coef., x*y coef, x*z coef, y*y coef, y*z coef, z*z coef 


#include "quadricfitting.h"

void outputQuadric(allquadrics::Quadric &q) {
    cout << q.q[0];
    for (int ii = 1; ii < 10; ii++) {
        cout << ", " << q.q[ii];
    }
}

int main(int argc, char* argv[])
{
	allquadrics::TriangleMesh inputMesh;

	char *defaultInput = "../example_data/cylinder.obj";
	char *meshfile =  argc > 1 ? argv[1] : 0;
	if (!meshfile) {
		cerr << "No mesh file specified?  Loading default mesh: " << defaultInput << endl;
		meshfile = defaultInput;
	}
	if (!inputMesh.loadObj(meshfile)) {
        cerr << "Couldn't load file " << meshfile << endl;
        return 1;
    }

	const char *quadricTypeNames[] = { "general", "rotationally symmetric", "plane", "sphere",
		"general cylinder", "circular cylinder", "general cone", "circular cone",
		"ellipsoid (BIASED METHOD)", "hyperboloid (BIASED METHOD)",
		"ellipsoid (IMPROVED)", "hyperboloid (IMPROVED)",
		"hyperboloid (1 sheet)", "hyperboloid (2 sheet)", "paraboloid", 
		"paraboloid (elliptical)", "paraboloid (hyperbolic)", 
		"elliptical cylinder", "hyperbolic cylinder", "parabolic cylinder" };

	
	// Always recenter and scale your data before fitting!
	inputMesh.centerAndScale(1);

	// EXAMPLE USAGE 1: fit all quadric types at once
    cout << "fitting using all fits function ..." << endl;
	vector<allquadrics::Quadric> qfits;
	fitAllQuadricTypes(inputMesh, qfits);

	for (size_t i = 0; i < qfits.size(); i++) {
		cout << "direct fit for quadric type: " << quadricTypeNames[i] << ":" << endl;
		outputQuadric(qfits[i]);
		cout << endl << endl;
	}
    
    // EXAMPLE USAGE 2: fit specific quadric types
    cout << "fitting using specific single-type functions ..." << endl;
    
    allquadrics::Quadric general;
    fitGeneralQuadric(inputMesh, general);
    cout << "direct fit for general quadric (unconstrained type): " << endl;
    outputQuadric(general);
    cout << endl << endl;
    
    allquadrics::Quadric sphere;
    fitSphere(inputMesh, sphere);
    cout << "direct fit for sphere: " << endl;
    outputQuadric(sphere);
    cout << endl << endl;
    
    allquadrics::Quadric ellipsoid;
    fitEllipsoid(inputMesh, ellipsoid);
    cout << "direct fit for ellipsoid: " << endl;
    outputQuadric(ellipsoid);
    cout << endl << endl;
    
    allquadrics::Quadric hyperboloid;
    fitHyperboloid(inputMesh, hyperboloid);
    cout << "direct fit for hyperboloid: " << endl;
    outputQuadric(hyperboloid);
    cout << endl << endl;
    
    allquadrics::Quadric generalCone;
    fitGeneralCone(inputMesh, generalCone, true);
    cout << "direct fit for general cone: " << endl;
    outputQuadric(generalCone);
    cout << endl << endl;
    
    allquadrics::Quadric circularCone;
    fitCircularCone(inputMesh, circularCone);
    cout << "direct fit for circular cone: " << endl;
    outputQuadric(circularCone);
    cout << endl << endl;
    
    allquadrics::Quadric generalCylinder;
    fitGeneralCylinder(inputMesh, generalCylinder);
    cout << "direct fit for general cylinder: " << endl;
    outputQuadric(generalCylinder);
    cout << endl << endl;
    
    allquadrics::Quadric circularCylinder;
    fitCircularCylinder(inputMesh, circularCylinder);
    cout << "direct fit for circular cylinder: " << endl;
    outputQuadric(circularCylinder);
    cout << endl << endl;

    // EXAMPLE USAGE 3: filter & fit only chosen triangles
    
    cout << "fitting using a selected subset of the input shape ..." << endl;
    
    // set triangle tags on mesh to activate 'selection mode'
    inputMesh.triangleTags.resize(inputMesh.triangles.size(), 0);
    // choose triangles with tag == 1 as the active selection, to be used in fitting
    inputMesh.activeTag = 1;
    // randomly select some of those triangles
    inputMesh.triangleTags[rand() % inputMesh.triangles.size()] = 1;
    inputMesh.triangleTags[rand() % inputMesh.triangles.size()] = 1;
    inputMesh.triangleTags[rand() % inputMesh.triangles.size()] = 1;
    inputMesh.triangleTags[rand() % inputMesh.triangles.size()] = 1;
    // fit a general quadric to that selection
    allquadrics::Quadric generalSubset;
    fitGeneralQuadric(inputMesh, generalSubset);
    cout << "direct fit (on four random triangles of input) for general quadric (unconstrained type): " << endl;
    outputQuadric(generalSubset);
    cout << endl << endl;


	return 0;
}

