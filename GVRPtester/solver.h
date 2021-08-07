#ifndef SOLVER_H
#define SOLVER_H

#include "instance_util.h"
#define OUTPUT_DIRECTORY "./tests/"

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
	double exceededDist;
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
	double greedyQuality;
	int numVisitedClients; 	// total of clientes serviced for all vehicles
	double executionTime; 	// total execution time * quiza debe ser en  main? isntance
	std::vector<int> visitedCustomerNodes;	// array with visited clients (1=visited, 0=othervise)
	std::vector<vehicleSolution> vehicleRoutes;	// vector with solutions for each vehicle
public:
	vehicleSolution greedySearch();
	vehicleSolution tabuSearch(vehicleSolution greedySol);
	swapPair makeMovementPair(nodeKey node1, nodeKey node2);
	Node findNodeByType(nodeKey n);
	AFSDepotRouteInfo findRouteToDepot(double acumDist, double acumTime, double lon1, double lat1);
	std::string routeToString(std::vector<nodeKey> r);
	int isTabu(std::vector<swapPair> tabu, swapPair movement);
	double calculateExceededDistance(vehicleSolution s);
	bool isValidSolution(vehicleSolution s);
	void setExecTime(double seconds);
	void writeSolution();
	void recalculateTimeQuality(vehicleSolution* s, int i, int j);
	void generateNewSol(vehicleSolution* s, int i, int j);
	GVRPSolver(Instance* instance);
	~GVRPSolver();
};	

#endif