#ifndef SOLVER_H
#define SOLVER_H

#include "instance_util.h"

#define OUTPUT_DIRECTORY "./solutions/"

double distanceHarvesine(double lon1, double lat1, double lon2, double lat2);

class GVRPSolver {
private:
	Instance* curInstance;	// current instance
	std::string solDir;		// solution directory
	double solQuality;		// solution quality == total distance for all vehicles
	int numVehicle; 		// final vehicle quantity
	int numVisitedClients; 	// total of clientes serviced for all vehicles
	double executionTime; 	// total execution time * quiza debe ser en  main? isntance
public:
	GVRPSolver(Instance* instance);
	~GVRPSolver();
};	

#endif