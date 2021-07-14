#include "solver.h"
#include <cmath>
using namespace std;

GVRPSolver::GVRPSolver(Instance* instance){
	cout << "Creando el solver..." << endl;
	curInstance = instance;
	solDir = OUTPUT_DIRECTORY;
	solQuality = 0;
	numVehicle = 0;
	numVisitedClients = 0;
	executionTime = 0; // check this
	// anything else? check later
};


GVRPSolver::~GVRPSolver(){

};

double distanceHarvesine(double lon1, double lat1, double lon2, double lat2) {
	double phi, lambda, r, toRadian, insideRootValue, d;
	toRadian = M_PI/180.0;  	// radian conversion constant
	r = 4182.44949;				// earth radius
	// pre-calculate formulae parts
	phi = ((lat2 - lat1) * toRadian)/2;
	phi = sin(phi);

	lambda = ((lon2 - lon1) * toRadian) / 2;
	lambda = sin(lambda);

	lat1 = (lat1) * toRadian;
    lat2 = (lat2) * toRadian;
    // using the Harvesine Distance formulae:
    insideRootValue = phi * phi + cos(lat1) * cos(lat2) * lambda * lambda;
    d = 2 * r * asin(sqrt(insideRootValue));
    return d;
}