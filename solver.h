#ifndef SOLVER_H
#define SOLVER_H

#include "instance_util.h"

#define OUTPUT_DIRECTORY "./solutions/"

double distanceHarvesine(double lon1, double lat1, double lon2, double lat2);

class GVRPSolver {
private:
	Instance* curInstance;	// current instance
	std::string solDir;		// solution directory
	double solQuality;		// solution quality
	int numVehicle; 		// final vehicle quantity
	int numVisitedClients; 	// # of clientes serviced
	double executionTime; 	// total execution time
	// como guardo solucion por cada vehiculo?
	// a pensar seniores
public:
	GVRPSolver(Instance* instance);
	~GVRPSolver();
};	

#endif