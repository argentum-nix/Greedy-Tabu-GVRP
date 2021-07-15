#ifndef SOLVER_H
#define SOLVER_H

#include "instance_util.h"

#define OUTPUT_DIRECTORY "./solutions/"
typedef std::map<int,int> hash;
typedef std::pair<char, int> nodeKey;

double distanceHarvesine(double lon1, double lat1, double lon2, double lat2);
void printNodeKeyVector(std::vector<nodeKey> v);

class vehicleSolution {
public:
	int vehicleAcumTime;
	int vehicleClients;
	std::vector<nodeKey> route;
	vehicleSolution();
	~vehicleSolution();
};

class GVRPSolver {
private:
	Instance* curInstance;	// current instance
	std::string solDir;		// solution directory
	double solQuality;		// solution quality == total distance for all vehicles
	int numVehicle; 		// final vehicle quantity
	int numVisitedClients; 	// total of clientes serviced for all vehicles
	double executionTime; 	// total execution time * quiza debe ser en  main? isntance

	std::vector<int> visitedCustomerNodes;
	std::map<hash, int> distances;
	// (key, distance)
	// key is such as <idNode1,idNode2>
	// if <idNode> is 2i, its a customer with id=i
	// otherwise (2i+1) its a fuel with id=i
	std::vector<vehicleSolution> vehicleRoutes;
public:
	std::vector<nodeKey> greedySearch();
	GVRPSolver(Instance* instance);
	~GVRPSolver();
};	

#endif