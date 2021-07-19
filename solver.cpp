#include "solver.h"
#include <cmath>
#include <iomanip>
#include <sys/stat.h>
using namespace std;

GVRPSolver::GVRPSolver(Instance* instance){
	curInstance = instance;
	solDir = OUTPUT_DIRECTORY;
	solQuality = 0;
	numVehicle = 0;
	numVisitedClients = 0;
	executionTime = 0; 
	// Initialize the vector
	for(int i = 0; i < curInstance->numCustomers; i++) {
		visitedCustomerNodes.push_back(0);
	}
	vehicleSolution sol = greedySearch();
	vehicleSolution tabu;
	double greedyQuality = 0;
	while(sol.vehicleClients != 0) {
		tabu = tabuSearch(sol);
		vehicleRoutes.push_back(tabu);

		numVisitedClients += tabu.vehicleClients;
		numVehicle++;
		solQuality += tabu.vehicleSolQuality;
		greedyQuality += sol.vehicleSolQuality;

		sol = greedySearch();
	}
} 


void GVRPSolver::setExecTime(double seconds) {
	executionTime = seconds;
}

string GVRPSolver::routeToString(vector<nodeKey> r) {
	string route;
	for(size_t i = 0; i < r.size(); i++) {
		route += r[i].first;
		route += to_string(r[i].second);
		route += '-';
	}
	route.pop_back();
	return route;
}

void GVRPSolver::writeSolution() {
	mkdir(solDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	ofstream out(solDir + curInstance->name + ".out");
	if(out.is_open()) {
		out << solQuality << "   " << numVisitedClients << "   " << numVehicle << "   " << executionTime << "\n";
		for(size_t i = 0; i < vehicleRoutes.size(); i++) {
			isValidSolution(vehicleRoutes[i]);
			out << left << setw(70) << routeToString(vehicleRoutes[i].route);
			out << setw(10) << vehicleRoutes[i].vehicleSolQuality;
			out << setw(10) << vehicleRoutes[i].vehicleAcumTime;
			out << setw(10) << "0" << "\n";
		}
		out.close();
	}	
	else {
		cout << "Error writing to file!\n";
	}
}

vehicleSolution::vehicleSolution() {
	vehicleAcumTime = vehicleClients = vehicleSolQuality = 0;
}

void vehicleSolution::setVehicleSolution(std::vector<nodeKey> r, double time, double quality, int clients) {
	vehicleSolQuality = quality;
	route = r;
	vehicleAcumTime = time;
	vehicleClients = clients;
}

GVRPSolver::~GVRPSolver(){
}

vehicleSolution::~vehicleSolution() {
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
		subrouteDistance = 0;
		subrouteTime = 0;
		lon3 = auxNode.longitude;
		lat3 = auxNode.latitude;
		// distance between current node & AFS
		dist1 = distanceHarvesine(lon1, lat1, lon3, lat3);
		// can travel to this AFS
		if(dist1 + acumDist <= curInstance->maxDistance) {
			// have enough time to travel
			subrouteTime = dist1/curInstance->speed + curInstance->serviceTime;
			if(acumTime + subrouteTime < curInstance->maxTime) {
				// distance between chosen AFS and depot
				dist2 = distanceHarvesine(lon3, lat3, lon2, lat2);
				// can travel to the depot
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

vehicleSolution GVRPSolver::greedySearch() {
	vector<nodeKey> greedyRoute;
	Node curNode = curInstance->depot;
	Node nextNode, auxNode;

	nodeKey curKey = {curNode.nodeType, curNode.nodeID};
	greedyRoute.push_back(curKey);

	double qualityGreedy = 0, timeGreedy = 0;
	double curDistance = 0, acumulatedDistance = 0, toDepotDist = 0;
	double minFoundDistance = 9999999;
	double lat1, lon1, lat2, lon2, dLon, dLat;
	double auxTime = 0, curNodeTime = 0;
	double finalReturnTime = 0, finalReturnDist = 0;
	int flagRefuel = 1, flagTerminate = 1, flagCanReturn = 0;
	int totalClients = 0;
	AFSDepotRouteInfo returnInfo;

	// Depot longitude and latitude
	dLon = curNode.longitude;
	dLat = curNode.latitude;

	while(timeGreedy < curInstance->maxTime) {
		lon1 = curNode.longitude;
		lat1 = curNode.latitude;
		for(int i = 0; i < curInstance->numCustomers; i++) {
			auxNode = curInstance->customerNodes[i];
			if(visitedCustomerNodes[i] == 0) { // unvisited node
				lon2 = auxNode.longitude;
				lat2 = auxNode.latitude;
				curDistance = distanceHarvesine(lon1, lat1, lon2, lat2);
				
				if(curDistance < minFoundDistance && acumulatedDistance+curDistance < curInstance->maxDistance) {
					// Distance between cur node and depot
					toDepotDist = distanceHarvesine(dLon, dLat, lon2, lat2);
					returnInfo = findRouteToDepot(acumulatedDistance + curDistance, timeGreedy, lon2, lat2);
					//can return from this new node?
					if(returnInfo.first.first != -1) {
						flagCanReturn = 1;
						if(returnInfo.second.first + returnInfo.second.second < toDepotDist) {
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
								}
							}

						}
						else {
							// its cheaper to return to deposit directely
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
				if(returnInfo.first.first != 0) {
					finalReturnTime = returnInfo.second.second/curInstance->speed;
					finalReturnDist = returnInfo.second.second;
				}
				else {
					finalReturnTime = returnInfo.second.first/curInstance->speed;
					finalReturnDist = returnInfo.second.first;
				}
			}
		}
		if(flagTerminate) break;
		if(!flagCanReturn) break;

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
		if(nextNode.nodeType == 'c') totalClients++;
		greedyRoute.push_back({nextNode.nodeType, nextNode.nodeID});
	}

	// when last visited node is f0, dont count in the refuel time
	// exchange f0 with d0
	if(greedyRoute.back().first == 'f' && greedyRoute.back().second == 0){
		greedyRoute.pop_back();
		timeGreedy -= curInstance->refuelTime;
		finalReturnTime = 0;
		finalReturnDist = 0;
	}
	greedyRoute.push_back({'d', 0});

	timeGreedy += finalReturnTime;
	qualityGreedy += finalReturnDist;

	vehicleSolution solution;
	solution.setVehicleSolution(greedyRoute, timeGreedy, qualityGreedy, totalClients);
	return solution;
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


int GVRPSolver::isTabu(std::vector<swapPair> tabu, swapPair movement) {
	nodeKey node1, node2;
	nodeKey swap1, swap2;
	swap1 = movement.first;
	swap2 = movement.second;
	for(size_t i = 0; i < tabu.size(); i++) {
		node1 = tabu[i].first;
		node2 = tabu[i].second;
		if(node1.first == swap1.first && node1.second == swap1.second) {
			if(node2.first == swap2.first && node2.second == swap2.second) {
				return 1; // movement is tabu
			}
		}
	}
	return 0; // movement isnt tabu
}

Node GVRPSolver::findNodeByType(nodeKey n) {
	if(n.first == 'c') {
		return curInstance->customerNodes[n.second-1];
	}
	else if(n.first == 'f') {
		return curInstance->fuelNodes[n.second];
	}
	return curInstance->depot;
}

void GVRPSolver::recalculateTimeQuality(vehicleSolution* s, int i, int j) {
	double e1 = 0, e2= 0, e3 = 0, e4 = 0;
	double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
	double newQuality = s->vehicleSolQuality;
	double newTime = s->vehicleAcumTime;
	int n = j;
	int m = i;

	Node aux1, aux2, aux3, aux4;
	if(i < j) {
		n = i;
		m = j;
	}
	// if we fully reversed the array, no need to recalculate the costs.
	if(n == 0 && size_t(m) == s->route.size() - 1) return;


	if(n != 0 && size_t(m) == s->route.size() - 1) {
		aux1 = findNodeByType(s->route[n - 1]);
		aux2 = findNodeByType(s->route[n]);
		aux3 = findNodeByType(s->route[m]);

		e1 = distanceHarvesine(aux1.longitude, aux1.latitude, aux2.longitude, aux2.latitude);
		e4 = distanceHarvesine(aux1.longitude, aux1.latitude, aux3.longitude, aux3.latitude);
		t1 = e1/curInstance->speed;
		t4 = e4/curInstance->speed;
	}
	else if(n == 0 && size_t(m) != s->route.size() - 1){
		aux2 = findNodeByType(s->route[n]);
		aux3 = findNodeByType(s->route[m]);
		aux4 = findNodeByType(s->route[m + 1]);

		e2 = distanceHarvesine(aux3.longitude, aux3.latitude, aux4.longitude, aux4.latitude);
		e3 = distanceHarvesine(aux2.longitude, aux2.latitude, aux4.longitude, aux4.latitude);
		t2 = e2/curInstance->speed;
		t3 = e3/curInstance->speed;
	}
	else if(n != 0 && size_t(m) != s->route.size() - 1) {
		aux1 = findNodeByType(s->route[n - 1]);
		aux2 = findNodeByType(s->route[n]);
		aux3 = findNodeByType(s->route[m]);
		aux4 = findNodeByType(s->route[m + 1]);
		e1 = distanceHarvesine(aux1.longitude, aux1.latitude, aux2.longitude, aux2.latitude);
		e2 = distanceHarvesine(aux3.longitude, aux3.latitude, aux4.longitude, aux4.latitude);
		e3 = distanceHarvesine(aux2.longitude, aux2.latitude, aux4.longitude, aux4.latitude);
		e4 = distanceHarvesine(aux1.longitude, aux1.latitude, aux3.longitude, aux3.latitude);
		t1 = e1/curInstance->speed;
		t2 = e2/curInstance->speed;
		t3 = e3/curInstance->speed;
		t4 = e4/curInstance->speed;
	}
	
	newQuality -= e1 + e2;
	newQuality += e3 + e4;
	newTime -= t1 + t2;
	newTime += t3 + t4;

	s->vehicleSolQuality = newQuality;
	s->vehicleAcumTime = newTime;
}

void GVRPSolver::generateNewSol(vehicleSolution* s, int i, int j) {
	recalculateTimeQuality(s, i, j);
	if(i < j) {
		reverse(s->route.begin() + i, s->route.begin() + j + 1); //2opt
	}
	else{
		reverse(s->route.begin() + j, s->route.begin() + i + 1); //2opt;
	}
}

swapPair GVRPSolver::makeMovementPair(nodeKey node1, nodeKey node2) {
	swapPair p;
	p = {{node1.first, node1.second}, {node2.first, node2.second}};
	return p;
}

void printMovement(swapPair p){
	cout << "{{" << p.first.first << ", " << p.first.second << "},{";
	cout << p.second.first << ", " << p.second.second << "}}\n";
}

bool GVRPSolver::isValidSolution(vehicleSolution s) {
	// time limit surpassed
	if(s.vehicleAcumTime > curInstance->maxTime) return false;

	nodeKey firstNode = s.route[0];
	nodeKey lastNode = s.route[s.route.size() - 1];
	// do not start at the depot
	if(firstNode.first != 'd') return false;

	// do not return to the depot
	if(lastNode.first != 'd') return false;

	// check if distance was not violated
	Node auxNode, curNode;
	double totalQuality = 0, totalTime = 0, acumDist = 0, dist = 0;
	curNode = curInstance->depot;
	for(size_t i = 1; i < s.route.size(); i++) {
		if(s.route[i].first == 'c' ) {
			auxNode = curInstance->customerNodes[s.route[i].second-1];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist += dist;
			if(dist > curInstance->maxDistance || acumDist > curInstance->maxDistance) {
				return false;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed + curInstance->serviceTime;
		}
		else if(s.route[i].first == 'f') {
			auxNode = curInstance->fuelNodes[s.route[i].second];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist = 0;
			if(dist > curInstance->maxDistance) {
				return false;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed + curInstance->refuelTime;
		}
		else {
			auxNode = curInstance->depot;
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			if(dist > curInstance->maxDistance || acumDist > curInstance->maxDistance) {
				return false;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed;

		}
		curNode = auxNode;
	}
	return true;
}

vehicleSolution GVRPSolver::tabuSearch(vehicleSolution greedySol) {
	vehicleSolution sCurrent = greedySol;
	vehicleSolution sBest = sCurrent;

	vector<swapPair> tabu;
	size_t sizeTabu = 20;
	swapPair movement;

	double curQuality = sCurrent.vehicleSolQuality;

	int t = 30;
	while(t--) {
		for(size_t i = 0; i < greedySol.route.size(); i++) {
			for(size_t j = 0; j < greedySol.route.size(); j++) {
				if(int(j) - int(i) >= 2 || int(i) - int(j) >= 2) {
					movement = makeMovementPair(sCurrent.route[i], sCurrent.route[j]);
					if(!isTabu(tabu, movement)) {
						generateNewSol(&sCurrent, i, j);

						tabu.push_back(movement);
						if(tabu.size() > sizeTabu) {
							tabu.erase(tabu.begin());
						}

						if(isValidSolution(sCurrent) && sCurrent.vehicleSolQuality < curQuality) {
							sBest = sCurrent;
							curQuality = sCurrent.vehicleSolQuality;

						}
					}
				}
			}
		}
	}
	return sBest;
}

