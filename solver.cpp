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
	vector<nodeKey> xd = greedySearch();
	checkFeasibility(xd);
}

vehicleSolution::vehicleSolution(){
	vehicleAcumTime = vehicleClients = 0;
}


GVRPSolver::~GVRPSolver(){
}

vehicleSolution::~vehicleSolution(){
}

AFSDepotRouteInfo GVRPSolver::findRouteToDepot(double acumDist, double acumTime, double lon1, double lat1) {
	double minFoundDistance = 9999999;
	double extraTime = 0;
	double dist1, dist2;
	double lon2, lon3, lat2, lat3;
	double subrouteDistance = 0;
	double subrouteTime = 0;
	AFSDepotRouteInfo solution = {{-1, 0}, {0, 0}}; // default solution
	// Longitude & Latitude of the depot node
	lon2 = curInstance->depot.longitude;
	lat2 = curInstance->depot.latitude;
	Node auxNode;
	for(int i = 0; i < curInstance->numStations; i++) {
		auxNode = curInstance->fuelNodes[i];
		//cout<< "Entre con el nodo " << auxNode.nodeID << endl;
		subrouteDistance = 0;
		subrouteTime = 0;
		lon3 = auxNode.longitude;
		lat3 = auxNode.latitude;
		// distance between current node & AFS
		dist1 = distanceHarvesine(lon1, lat1, lon3, lat3);
		//DEBUG(dist1);
		//DEBUG(acumDist);
		//DEBUG(curInstance->maxDistance);
		// can travel to this AFS
		if(dist1 + acumDist <= curInstance->maxDistance) {
			// have enough time to travel
			subrouteTime = dist1/curInstance->speed + curInstance->serviceTime;
			if(acumTime + subrouteTime < curInstance->maxTime) {
				// distance between chosen AFS and depot
				dist2 = distanceHarvesine(lon3, lat3, lon2, lat2);
				// can travel to the depot
				//DEBUG(dist1);
				//DEBUG(dist2);
				//DEBUG(dist1+dist2);
				if(dist2 <= curInstance->maxDistance) {
					// just enough time to travel and finish the route
					subrouteTime += dist2/curInstance->speed;
					if(acumTime + subrouteTime <= curInstance->maxTime) {
						subrouteDistance = dist1 + dist2;
						if(subrouteDistance < minFoundDistance) {
							minFoundDistance = subrouteDistance; // minimum cost found
							extraTime = subrouteTime; // time it would take to travel
							solution =  {{auxNode.nodeID, extraTime}, {dist1, dist2}};
							// {AFS_Id, Time}, {Distance to the AFS, Distance from AFS to Depot}
						}
					}
				}
			}
		}
	}
	//cout << "((" << solution.first.first << ", " << solution.first.second << "), (" << solution.second.first << ", " << solution.second.second << "))\n"; 
	return solution;
}

vector<nodeKey> GVRPSolver::greedySearch() {
	double qualityGreedy = 0;
	double timeGreedy = 0;
	vector<nodeKey> greedyRoute;
	Node curNode = curInstance->depot;
	Node nextNode;

	Node auxNode;

	nodeKey curKey = {curNode.nodeType, curNode.nodeID};
	greedyRoute.push_back(curKey);

	double curDistance = 0;
	double acumulatedDistance = 0;
	double minFoundDistance = 9999999;
	double lat1, lon1, lat2, lon2, dLon, dLat;
	double auxTime = 0, curNodeTime = 0;
	int flagRefuel = 1, flagTerminate = 1;
	double toDepotDist = 0;
	AFSDepotRouteInfo returnInfo;
	// Depot longitude and latitude
	dLon = curNode.longitude;
	dLat = curNode.latitude;

	int flagAFSReturnRoute = 0;
	int flagDepotReturnRoute = 0;

	double finalReturnTime = 0, finalReturnDist = 0;
	int intermediateAFSId = 0, flagCanReturn = 0;

	// quiza es mejor un mientras hayan nodos de clientes a visitar.
	while(timeGreedy < curInstance->maxTime) {
		lon1 = curNode.longitude;
		lat1 = curNode.latitude;
		flagAFSReturnRoute = 0;
		flagDepotReturnRoute = 0;
		for(int i = 0; i < curInstance->numCustomers; i++) {
			auxNode = curInstance->customerNodes[i];
			if(visitedCustomerNodes[i] == 0) { // unvisited node
				lon2 = auxNode.longitude;
				lat2 = auxNode.latitude;
				curDistance = distanceHarvesine(lon1, lat1, lon2, lat2);
				//DEBUG(curDistance);
				//cout << auxNode.nodeType << auxNode.nodeID << ": ";
				
				if(curDistance < minFoundDistance && acumulatedDistance+curDistance < curInstance->maxDistance) {
					// tengo combustible?
					// Distance between cur node and depot
					toDepotDist = distanceHarvesine(dLon, dLat, lon2, lat2);
					returnInfo = findRouteToDepot(acumulatedDistance + curDistance, timeGreedy, lon2, lat2);
					//can return from this new node?
					if(returnInfo.first.first != -1) {
						flagCanReturn = 1;
						//DEBUG(toDepotDist);
						//DEBUG(returnInfo.second.first + returnInfo.second.second);
						if(returnInfo.second.first + returnInfo.second.second < toDepotDist) {
							flagAFSReturnRoute = 1;
							if(acumulatedDistance + curDistance + returnInfo.second.first <= curInstance->maxDistance) {
								auxTime = (curDistance / curInstance->speed) + curInstance->serviceTime;
								if(auxTime + timeGreedy + returnInfo.first.second <= curInstance->maxTime) {
									nextNode = auxNode;
									minFoundDistance = curDistance;
									curNodeTime = auxTime;
									flagRefuel = 0;
									flagTerminate = 0;
									finalReturnTime = returnInfo.first.second;
									finalReturnDist = returnInfo.second.first + returnInfo.second.second;
									intermediateAFSId = returnInfo.first.first;
								}
							}

						}
						else {
							// its cheaper to return to deposit directely
							flagDepotReturnRoute = 1;
							if(acumulatedDistance + curDistance + toDepotDist <= curInstance->maxDistance) {
								auxTime = (curDistance / curInstance->speed) + curInstance->serviceTime;
								if(auxTime + timeGreedy + toDepotDist / curInstance->speed <= curInstance->maxTime) {
									nextNode = auxNode;
									minFoundDistance = curDistance;
									curNodeTime = auxTime;
									flagRefuel = 0;
									flagTerminate = 0;
									finalReturnDist = toDepotDist;
									finalReturnTime = toDepotDist / curInstance->speed;
								}
							}
						}
					}
				}
			}
		}
		// find a refuel station if cant travel to any client 
		// and only if the last node visited WAS NOT a station
		if(flagRefuel && greedyRoute.back().first != 'f') {
			//cout << "Parece que no puedo viajar a ningun nodo, buscare un refuel\n";
			returnInfo = findRouteToDepot(acumulatedDistance, timeGreedy, curNode.longitude, curNode.latitude);
			if(returnInfo.first.first != -1) {
				flagCanReturn = 1;
				nextNode = curInstance->fuelNodes[returnInfo.first.first];
				minFoundDistance = returnInfo.second.first;
				curNodeTime = curInstance->refuelTime + returnInfo.second.first / curInstance->speed;
				flagTerminate = 0;
				acumulatedDistance = 0;
				cout << returnInfo.first.first << " " << returnInfo.first.second << " " << returnInfo.second.first << " " << returnInfo.second.second << endl;
				if(returnInfo.first.first != 0) {
					finalReturnTime = returnInfo.second.second/curInstance->speed;
					finalReturnDist = returnInfo.second.second;
				}
				else {
					finalReturnTime = returnInfo.second.first/curInstance->speed;
					finalReturnDist = returnInfo.second.first;
				}
				

			}
			else {
				cout << "Intente buscar un refuel y no encontre ninguno, -1, xd\n";
			}
		}
		if(flagTerminate) break;


		//DEBUG!!!
		if(!flagCanReturn) {
			cout << "No hay ningun nodo para poder volver a la casita" << endl;
			break;
		}

		cout << "\nEl minimo que encontre fue el nodo " << nextNode.nodeType << nextNode.nodeID << " con distancia minima= " << minFoundDistance << endl;
		//DEBUG(acumulatedDistance);
		if(!flagRefuel) {
			visitedCustomerNodes[nextNode.nodeID-1] = 1;
		}
		timeGreedy += curNodeTime;
		acumulatedDistance += minFoundDistance;
		qualityGreedy += minFoundDistance;
		acumulatedDistance += minFoundDistance;
		minFoundDistance = 9999999;
		flagRefuel = 1;
		flagTerminate = 1;
		flagCanReturn = 0;
		curNode = nextNode;
		greedyRoute.push_back({nextNode.nodeType, nextNode.nodeID});
		//cout << nextNode.nodeType << nextNode.nodeID << " " << qualityGreedy << " " << timeGreedy << "\n";
	}

	// when last visited node is f0, dont count in the refuel time
	// exchange f0 with d0
	if(greedyRoute.back().first == 'f' && greedyRoute.back().second == 0){
		cout<< "entreeeee\n";
		greedyRoute.pop_back();
		timeGreedy -= curInstance->refuelTime;
		finalReturnTime = 0;
		finalReturnDist = 0;
	}
	greedyRoute.push_back({'d', 0});

	timeGreedy += finalReturnTime;
	qualityGreedy += finalReturnDist;

	DEBUG(qualityGreedy);
	DEBUG(timeGreedy);
	printNodeKeyVector(greedyRoute);
	return greedyRoute;
}

void GVRPSolver::checkFeasibility(std::vector<nodeKey> v) {
	Node auxNode, curNode;
	double totalQuality = 0;
	double totalTime = 0;
	double acumDist = 0;
	double dist = 0;
	cout << "CHECK:==========================================\n";
	curNode = curInstance->depot;
	for(size_t i = 1; i < v.size(); i++) {
		if(v[i].first == 'c' ) {
			auxNode = curInstance->customerNodes[v[i].second-1];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist += dist;
			if(dist > curInstance->maxDistance || acumDist > curInstance->maxDistance) {
				cout << "No se cumplio la restriccion de viaje maximo en el nodo: " << auxNode.nodeType << auxNode.nodeID << endl;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed + curInstance->serviceTime;
		}
		else if(v[i].first == 'f') {
			auxNode = curInstance->fuelNodes[v[i].second];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist = 0;
			if(dist > curInstance->maxDistance) {
				cout << "No se cumplio la restriccion de viaje maximo en el nodo: " << auxNode.nodeType << auxNode.nodeID << endl;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed + curInstance->refuelTime;
		}
		else {
			auxNode = curInstance->depot;
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			if(dist > curInstance->maxDistance || acumDist > curInstance->maxDistance) {
				cout << "No se cumplio la restriccion de viaje maximo en el nodo: " << auxNode.nodeType << auxNode.nodeID << endl;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed;

		}
		curNode = auxNode;
		if(totalTime > curInstance->maxTime) {
			cout << "No se cumplio la restriccion de tiempo en el nodo: " << auxNode.nodeType << auxNode.nodeID << endl;
		}

		//cout << auxNode.nodeType << auxNode.nodeID << " " << totalQuality << " " << totalTime << "\n";

	}
	DEBUG(totalTime);
	DEBUG(totalQuality);
}

void printNodeKeyVector(std::vector<nodeKey> v) {
	for(auto x: v) {
		cout << x.first << x.second << "-";
	}
	cout << "*\n";
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