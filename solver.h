#ifndef SOLVER_H
#define SOLVER_H

#include "instance_util.h"

#define OUTPUT_DIRECTORY "./solutions/"
//typedef std::map<int,int> hash;
typedef std::pair<char, int> nodeKey;
typedef std::pair<std::pair<int, double>, std::pair<double, double>> AFSDepotRouteInfo;
typedef std::pair<nodeKey, nodeKey> swapPair;

double distanceHarvesine(double lon1, double lat1, double lon2, double lat2);
void printNodeKeyVector(std::vector<nodeKey> v);
void printMovement(swapPair p);

class vehicleSolution {
public:
	double vehicleAcumTime;
	double vehicleSolQuality;
	int vehicleClients;
	std::vector<nodeKey> route;

	void setVehicleSolution(std::vector<nodeKey> r, double time, double quality, int clients);
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
	std::vector<vehicleSolution> vehicleRoutes;
public:
	vehicleSolution greedySearch();
	vehicleSolution tabuSearch(vehicleSolution greedySol);
	AFSDepotRouteInfo findRouteToDepot(double acumDist, double acumTime, double lon1, double lat1);
	int isTabu(std::vector<swapPair> tabu, swapPair movement);
	void recalculateTimeQuality(vehicleSolution* s, int i, int j);
	void generateNewSol(vehicleSolution* s, int i, int j);
	swapPair makeMovementPair(nodeKey node1, nodeKey node2);
	Node findNodeByType(nodeKey n);
	bool isValidSolution(vehicleSolution s);
	GVRPSolver(Instance* instance);
	~GVRPSolver();
};	

#endif