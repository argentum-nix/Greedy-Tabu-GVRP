#include "instance_util.h"
#include <fstream>
#include <sstream>
#include <algorithm>
using namespace std;

Instance::Instance(std::string iName) {
	dir = TARGET_DIRECTORY;
	name = iName;
}

Graph::Graph() {
	size = 0;
}

Node::Node(int id, char t, double lon, double lat){
	nodeID = id;
	nodeType = t;
	longitude = lon;
	latitude = lat;
}

void printVector(vector<string> words){
	for(size_t i = 0; i < words.size(); i++) {
		cout << words[i] << endl;
	}
}


void Instance::loadData() {
	ifstream in(dir + name + ".dat");
	if(!in.is_open()) throw 20;
	streambuf *cinbuf = cin.rdbuf();
    cin.rdbuf(in.rdbuf());

    int id;
    char ntype;
    double longitude, latitude;
    // Lee la primera linea que es distinta
    cin >> name
    	>> numCustomers
    	>> numStations
    	>> maxTime
    	>> maxDistance
    	>> speed
    	>> serviceTime
    	>> refuelTime;

   
    /*DEBUG(name);
    DEBUG(numStations);
    DEBUG(numStations);
    DEBUG(maxTime);
    DEBUG(maxDistance);
    DEBUG(speed);
    DEBUG(serviceTime);
    DEBUG(refuelTime);*/

 	Graph graph;
 	// Lee el arhivo restante
    while(cin >> id) {
    	cin >> ntype 
    		>> longitude
    		>> latitude;
    	
    	/*cout << "================\n";
    	DEBUG(id);
    	DEBUG(ntype);
    	DEBUG(longitude);
    	DEBUG(latitude);
    	cout << "================\n";*/

    	// Crea el nodo y lo agrega en el diccionario
    	// cuya llave es (tipo de nodo, id)
    	Node node(id, ntype, longitude, latitude);
    	Key key(ntype, id);
    	nodeMap.insert({key, node});
    	// aqui hay que agregar el nodo a la lista de adyacencia
    }
    cin.rdbuf(cinbuf);
    in.close();
}



Instance::~Instance() {
	//cout << "[LOG] Instance destroyed\n";
}

Node::~Node() {
	//cout << "[LOG] Node destroyed\n";
}

Graph::~Graph() {
	//cout << "[LOG] Graph destroyed\n";
}