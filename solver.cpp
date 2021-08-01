#include "solver.h"
#include <cmath>
#include <iomanip>
#include <sys/stat.h>
using namespace std;

// GVRPSolver class constructor
//======================================================================
// Constructor de solver. Además, posee el ciclo que intenta enviar un 
// nuevo vehiculo, hasta que se cumpla que algun vehiculo tenga la ruta
// d0-d0 asignada (0 clientes). En este caso se termina la asignacion 
// de vehiculos. Cada ruta es mejorada a traves de TS.
//======================================================================
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

// void GVRPSolver::setExecTime
//======================================================================
// Setter de tiempo de ejecucion para GVRPSolver.
//======================================================================
// Returns: void
void GVRPSolver::setExecTime(double seconds) {
	executionTime = seconds;
}

// string GVRPSolver::routeToString
//======================================================================
// Funcion toString para la ruta (tour) de vehiculo
//======================================================================
// Returns: string, ruta en formato string
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

// double GVRPSolver::calculateExceededDistance
//======================================================================
// En caso de que flag exceededDist no es -1, existe distancia excedida
// durante el viaje, que se calcula de manera lineal. Debido a las 
// restricciones duras, se espera que el flag exceededDist siempre sea -1.
//======================================================================
// Returns: double, distancia excedida
double GVRPSolver::calculateExceededDistance(vehicleSolution s) {
	if(s.exceededDist != -1) return 0;
	Node auxNode, curNode;
	double exceeded = 0, acumDist = 0, dist = 0;
	curNode = curInstance->depot;
	for(size_t i = 1; i < s.route.size(); i++) {
		if(s.route[i].first == 'c' ) {
			auxNode = curInstance->customerNodes[s.route[i].second-1];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist += dist;
			if(dist > curInstance->maxDistance) {
				exceeded += dist - curInstance->maxDistance;
			}
			else if(acumDist > curInstance->maxDistance) {
				exceeded += dist - curInstance->maxDistance;
			}
		}
		else if(s.route[i].first == 'f') {
			auxNode = curInstance->fuelNodes[s.route[i].second];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist = 0;
			if(dist > curInstance->maxDistance) {
				exceeded += dist - curInstance->maxDistance;
			}
		}
		else {
			auxNode = curInstance->depot;
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			if(dist > curInstance->maxDistance) {
				exceeded += dist - curInstance->maxDistance;
			}
			else if(acumDist > curInstance->maxDistance) {
				exceeded += dist - curInstance->maxDistance;
			}
		}
		curNode = auxNode;
	}
	return exceeded;
}

// void GVRPSolver::writeSolution
//======================================================================
// Escribe la solucion final al archivo de tipo AB{}.out
// Usando mkdir se crea la carpeta solutions, en caso de correr el 
// programa varias veces, se debe borrar su contenido ya que no sera 
// sobreescrito. O se puede borrar la carpeta entera.
//======================================================================
// Returns: void
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
			out << setw(10) << calculateExceededDistance(vehicleRoutes[i]) << "\n";
		}
		out.close();
	}	
	else {
		cout << "Error writing to file!\n";
	}
}

// vehicleSolution class constructor
vehicleSolution::vehicleSolution() {
	vehicleAcumTime = vehicleClients = vehicleSolQuality = 0;
}

// void vehicleSolution::setVehicleSolution
//======================================================================
// Setea los campos de la solucion para un vehiculo especifico.
//======================================================================
// Returns: void
void vehicleSolution::setVehicleSolution(std::vector<nodeKey> r, double time, double quality, int clients) {
	vehicleSolQuality = quality;
	route = r;
	exceededDist = 0;
	vehicleAcumTime = time;
	vehicleClients = clients;
}

// GVRPSolver class destructor
GVRPSolver::~GVRPSolver(){
}

// vehicleSolution class destructor
vehicleSolution::~vehicleSolution() {
}

// AFSDepotRouteInfo GVRPSolver::findRouteToDepot
//======================================================================
// Busca una ruta desde el nodo actual hacia el depot
// 1) Puede ser una ruta directa nodo->depot
// 2) Puede ser una ruta con AFS intermediario: nodo->AFS->depot
//======================================================================
// Returns: AFSDepotRouteInfo, par de pares con informacion de ruta al depot
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
		// Distance between current node & AFS
		dist1 = distanceHarvesine(lon1, lat1, lon3, lat3);
		// Can travel to this AFS?
		if(dist1 + acumDist <= curInstance->maxDistance) {
			// We have enough time to travel
			subrouteTime = dist1/curInstance->speed + curInstance->serviceTime;
			if(acumTime + subrouteTime < curInstance->maxTime) {
				// Distance between chosen AFS and depot
				dist2 = distanceHarvesine(lon3, lat3, lon2, lat2);
				// Can  travel to the depot?
				if(dist2 <= curInstance->maxDistance) {
					// Just enough time to travel and finish the route?
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
	return solution;
}

// vehicleSolution GVRPSolver::greedySearch
//======================================================================
// Rutina de Greedy Search. Se selecciona el arco de menor distancia
// al nodo actual, que cumple con las restricciones duras impuestas.
// 1) El nodo puede ser agregado a la solucion si desde este existe una ruta
// que permite volver al depot - sea esta ruta directa, o a traves de un AFS
// 2) Se debe cumplir la restriccion de viaje internodal (no viajar mas que)
// el estanque permita
// 3) Se debe cumplir la restriccion de tiempo total de vehiculo
//
// Se prefieren siempre nodos cliente por sobre nodos AFS. Solo en caso de no
// poder agregar un nuevo cliente (por no cumplir restriccion de viaje), 
// se busca AFS que cumpla con restricciones 1,2,3. En caso de no poder agregar
// ningun otro nodo, se retorna.
//
// La funcion retorna una solucion factible que cumpla con las restricciones
// mencionadas.
//======================================================================
// Returns: vehicleSolution, solucion Greedy factible obtenida
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
			// If node is unvisited
			if(visitedCustomerNodes[i] == 0) {
				lon2 = auxNode.longitude;
				lat2 = auxNode.latitude;
				curDistance = distanceHarvesine(lon1, lat1, lon2, lat2);
				
				if(curDistance < minFoundDistance && acumulatedDistance+curDistance < curInstance->maxDistance) {
					// Distance between cur node and depot
					toDepotDist = distanceHarvesine(dLon, dLat, lon2, lat2);
					returnInfo = findRouteToDepot(acumulatedDistance + curDistance, timeGreedy, lon2, lat2);
					// Can we return from this new node to depot by some route?
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
							// Its cheaper to return to deposit directely than by AFS->depot route
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
		// Find a refuel station if cant travel to any client 
		// and only if the last node visited WAS NOT a station
		if(flagRefuel && greedyRoute.back().first != 'f') {
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
	// When last visited node is f0, dont count in the refuel time
	// exchange f0 with d0
	if(greedyRoute.back().first == 'f' && greedyRoute.back().second == 0) {
		greedyRoute.pop_back();
		timeGreedy -= curInstance->refuelTime;
		finalReturnTime = 0;
		finalReturnDist = 0;
	}
	greedyRoute.push_back({'d', 0});
	timeGreedy += finalReturnTime;
	qualityGreedy += finalReturnDist;
	// Set the solution
	vehicleSolution solution;
	solution.setVehicleSolution(greedyRoute, timeGreedy, qualityGreedy, totalClients);
	return solution;
}

// void printNodeKeyVector
//======================================================================
// Funcion de debugging. Imprime el vector de tipo nodeKey - el tour 
// que toma el vehiculo.
//======================================================================
// Returns: void
void printNodeKeyVector(std::vector<nodeKey> v) {
	for(auto x: v) {
		cout << x.first << x.second << "-";
	}
	cout << "*\n";
}

// double distanceHarvesine
//======================================================================
// La funcion obtiene la distancia harvesiana para dos puntos en el espacio.
//======================================================================
// Returns: double, distancia harvesiana
double distanceHarvesine(double lon1, double lat1, double lon2, double lat2) {
	double phi, lambda, r, toRadian, insideRootValue, d;
	toRadian = M_PI / 180.0;  	// radian conversion constant
	r = 4182.44949;				// earth radius
	// Pre-calculate formulae parts
	phi = ((lat2 - lat1) * toRadian)/2;
	phi = sin(phi);

	lambda = ((lon2 - lon1) * toRadian) / 2;
	lambda = sin(lambda);

	lat1 = (lat1) * toRadian;
    lat2 = (lat2) * toRadian;
    // Using the Harvesine Distance formulae:
    insideRootValue = phi * phi + cos(lat1) * cos(lat2) * lambda * lambda;
    d = 2 * r * asin(sqrt(insideRootValue));
    return d;
}

// int GVRPSolver::isTabu
//======================================================================
// La funcion realiza una busqueda de un par 'movement' en el vector de
// pares tabu para revisar si el movimiento ya se ubica en la lista, 
// es decir, si el movimiento es tabu.
//======================================================================
// Returns: int, 1 si el movimiento es Tabu, 0 en caso contrario 
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
	return 0; // movement isn't tabu
}

// Node GVRPSolver::findNodeByType
//======================================================================
// La funcion obtiene el nodo a traves de su representacion de par
// nodeKey. Por ejemplo, nodeKey {'c', '1'} corresponde a la instancia
// de clase Node que se ubica en la posicion 0 del arreglo de nodos de 
// clientes.
//======================================================================
// Returns: Node, el nodo buscado por su llave nodeKey 
Node GVRPSolver::findNodeByType(nodeKey n) {
	if(n.first == 'c') {
		return curInstance->customerNodes[n.second-1];
	}
	else if(n.first == 'f') {
		return curInstance->fuelNodes[n.second];
	}
	return curInstance->depot;
}

// void GVRPSolver::recalculateTimeQuality
//======================================================================
// Recalcula costos de arcos reconectados despues de aplicar 2opt.
// En mejor caso se deben recalcular 2 arcos, en peor caso se recalculan
// 4 arcos. La funcion cambia el valor de calidad y tiempo de la
// solucion actual.
//======================================================================
// Returns: void
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
	// If we fully reversed the array, there is no need to recalculate the costs.
	if(n == 0 && size_t(m) == s->route.size() - 1) return;

	// Recalculating the reconnected 2opt edges
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

// void GVRPSolver::generateNewSol
//======================================================================
// Aplica la inversion de la ruta entre los nodos de indices i y j para 2opt.
//======================================================================
// Returns: void
void GVRPSolver::generateNewSol(vehicleSolution* s, int i, int j) {
	recalculateTimeQuality(s, i, j);
	if(i < j) {
		reverse(s->route.begin() + i, s->route.begin() + j + 1); //2opt
	}
	else{
		reverse(s->route.begin() + j, s->route.begin() + i + 1); //2opt;
	}
}

// swapPair GVRPSolver::makeMovementPair
//======================================================================
// Crea el par de pares para representar un movimiento. Un ejemplo
// de movimiento seria un par {{'d', '0'}, {'c', '1'}} que indica que se
// hizo 2opt entre nodos d0 y c1.
//======================================================================
// Returns: swapPair, un par de pares
swapPair GVRPSolver::makeMovementPair(nodeKey node1, nodeKey node2) {
	swapPair p;
	p = {{node1.first, node1.second}, {node2.first, node2.second}};
	return p;
}

// void printMovement
//======================================================================
// Funcion de debugging. Imprime el movimiento de tipo swapPair.
//======================================================================
// Returns: void
void printMovement(swapPair p){
	cout << "{{" << p.first.first << ", " << p.first.second << "},{";
	cout << p.second.first << ", " << p.second.second << "}}\n";
}


// bool GVRPSolver::isValidSolution
//======================================================================
// Verifica si la solucion es valida, considerando tres restricciones duras
// 1) Inicio y Fin de tour en el deposito d0
// 2) Distancia de viaje internodal sin recarga
// 3) Tiempo total de uso de vehiculo
//
// Si no se viola la distancia de viaje, exceededDist se setea en -1
// para optimizar - indica que no se debe recalcular la distancia excedida
// ya que esta es nula. En caso contrario, durante la fase de output
// se recalculara la distancia excedida.
//======================================================================
// Returns: true si la solucion es valida, false etoc
bool GVRPSolver::isValidSolution(vehicleSolution s) {
	// Time limit surpassed
	if(s.vehicleAcumTime > curInstance->maxTime) return false;

	nodeKey firstNode = s.route[0];
	nodeKey lastNode = s.route[s.route.size() - 1];
	// Tour does not start at the depot
	if(firstNode.first != 'd') return false;

	// Tour does not end at the depot
	if(lastNode.first != 'd') return false;

	// Check if distance restriction wasn't violated
	Node auxNode, curNode;
	double totalQuality = 0, totalTime = 0, acumDist = 0, dist = 0;
	curNode = curInstance->depot;
	for(size_t i = 1; i < s.route.size(); i++) {
		// Case 1: client node
		if(s.route[i].first == 'c' ) {
			auxNode = curInstance->customerNodes[s.route[i].second-1];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist += dist;
			if(dist > curInstance->maxDistance || acumDist > curInstance->maxDistance) {
				s.exceededDist = -1; // flag -1, the algorithm won't calculate excedeed distance to optimize
				return false;		 // and return directly without looping over the whole route
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed + curInstance->serviceTime;
		}
		// Case 2: fuel node
		else if(s.route[i].first == 'f') {
			auxNode = curInstance->fuelNodes[s.route[i].second];
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			acumDist = 0;
			if(dist > curInstance->maxDistance) {
				s.exceededDist = -1;
				return false;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed + curInstance->refuelTime;
		}
		// Case 3: depot node
		else {
			auxNode = curInstance->depot;
			dist = distanceHarvesine(curNode.longitude, curNode.latitude, auxNode.longitude, auxNode.latitude);
			if(dist > curInstance->maxDistance || acumDist > curInstance->maxDistance) {
				s.exceededDist = -1;
				return false;
			}
			totalQuality += dist;
			totalTime += dist/curInstance->speed;

		}
		curNode = auxNode;
	}
	return true;
}

// vehicleSolution GVRPSolver::tabuSearch
//======================================================================
// Utiliza la solucion inicial generada por Greedy Search 
// El movimiento aplicado a tour es 2-opt 
// Largo de lista tabu (sizeTabu) y numero de iteraciones (t) 
// son los mejores en promedio encontrados durante la fase de experimentacion
//======================================================================
// Returns: la mejor solucion obtenida por TS
vehicleSolution GVRPSolver::tabuSearch(vehicleSolution greedySol) {
	vehicleSolution sCurrent = greedySol;
	vehicleSolution Sv = greedySol;
	vehicleSolution sBest = sCurrent;

	vector<swapPair> tabu;
	size_t sizeTabu = 13;
	swapPair movement;
	swapPair best_movement;
	double bestNeighborQuality;

	int t = 55;
	while(t--) {
		// Generate best neighbor
		bestNeighborQuality = 9999999;
		for(size_t i = 0; i < greedySol.route.size(); i++) {
			for(size_t j = 0; j < greedySol.route.size(); j++) {
				if(int(j) - int(i) >= 2 || int(i) - int(j) >= 2) {
					movement = makeMovementPair(Sv.route[i], Sv.route[j]);
					if(!isTabu(tabu, movement)) {
						generateNewSol(&Sv, i, j);
						if(isValidSolution(Sv) && Sv.vehicleSolQuality < bestNeighborQuality) {
							sCurrent = Sv;
							best_movement = movement;
							bestNeighborQuality = Sv.vehicleSolQuality;

						}
					}
					Sv = sCurrent;
				}
			}
		}
		// Update tabu
		tabu.push_back(best_movement);
		if(tabu.size() > sizeTabu) {
			tabu.erase(tabu.begin());
		}
		// Save best solution
		if(sCurrent.vehicleSolQuality < sBest.vehicleSolQuality) {
			sBest = sCurrent;
		}
	}
	return sBest;
}

