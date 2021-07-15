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
	// Initialize the vector
	for(int i = 0; i < curInstance->numCustomers; i++) {
		visitedCustomerNodes.push_back(0);
	}
	greedySearch();
	cout << "dps de search siii\n";
}

vehicleSolution::vehicleSolution(){
	vehicleAcumTime = vehicleClients = 0;
}


GVRPSolver::~GVRPSolver(){
}

vehicleSolution::~vehicleSolution(){
}

vector<nodeKey> GVRPSolver::greedySearch() {
	int qualityGreedy = 0;
	int timeGreedy = 0;
	vector<nodeKey> greedyRoute;
	Node curNode = curInstance->depot;
	Node nextNode;

	Node auxNode;

	nodeKey curKey = {curNode.nodeType, curNode.nodeID};
	greedyRoute.push_back(curKey);

	double curDistance = 0;
	double acumulatedDistance = 0;
	double minFoundDistance = 9999999;

	double lat1, lon1, lat2, lon2;

	double auxTime = 0;
	double curNodeTime = 0;

	while(timeGreedy < curInstance->maxTime) {
		for(int i = 0; i < curInstance->numCustomers; i++) {
			auxNode = curInstance->customerNodes[i];
			if(visitedCustomerNodes[i] == 0) { // unvisited node
				lon1 = curNode.longitude;
				lat1 = curNode.latitude;
				lon2 = auxNode.longitude;
				lat2 = auxNode.latitude;
				curDistance = distanceHarvesine(lon1, lat1, lon2, lat2);

				cout << auxNode.nodeType << auxNode.nodeID << ":";
				DEBUG(curDistance);
				DEBUG(minFoundDistance);

				if(curDistance < minFoundDistance) {
					// tengo combustible?
					if(acumulatedDistance + curDistance < curInstance->maxDistance) {
						auxTime = (curDistance / curInstance->speed) + curInstance->serviceTime;
						if(auxTime +timeGreedy < curInstance->maxTime) {
							nextNode = auxNode;
							minFoundDistance = curDistance;
							curNodeTime = auxTime;
						}
					}
					// no tengo combustible, debo recargar
					else {
						cout << "No puedo viajar al nodo customer id=" << i+1 << endl;
					}
				}
			}
		}
		cout << "===========================================\n";
		cout << "El minimo que encontre fue el nodo " << nextNode.nodeType << nextNode.nodeID << " con distancia minima= " << minFoundDistance << endl;;
		timeGreedy += curNodeTime;
		qualityGreedy += minFoundDistance;
		acumulatedDistance += minFoundDistance;
		DEBUG(acumulatedDistance);
		visitedCustomerNodes[nextNode.nodeID-1] = 1;
		minFoundDistance = 9999999;
		curNode = nextNode;
	}
	return greedyRoute;
}



double distanceHarvesine(double lon1, double lat1, double lon2, double lat2) {
	double phi, lambda, r, toRadian, insideRootValue, d;
	toRadian = M_PI / 180.0;  	// radian conversion constant
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