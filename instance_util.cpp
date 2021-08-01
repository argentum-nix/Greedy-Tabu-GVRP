#include <sstream>
#include "instance_util.h"
using namespace std;

// Node class constructor
Node::Node() {}

// void Node::setNodeData
//======================================================================
// Setter datos para el nodo cualquiera
//======================================================================
// Returns: void
void Node::setNodeData(int id, char t, double lon, double lat) {
	nodeID = id;
	nodeType = t;
	longitude = lon;
	latitude = lat;
}

// vehicleSolution class destructor
Node::~Node() {}

// Instance class constructor
Instance::Instance(std::string iName) {
	dir = TARGET_DIRECTORY;
	name = iName;
}

// Instance class destructor
Instance::~Instance() {}

// int Instance::loadData
//======================================================================
// Lector de datos de archivos de las instancias.
//======================================================================
// Returns: int, -1 si hubo error en lectura de datos, 0 etoc
int Instance::loadData() {
	ifstream in(dir + name + ".dat");
	if(in.is_open()) {
		streambuf *cinbuf = cin.rdbuf();
	    cin.rdbuf(in.rdbuf());

	    int id;
	    char ntype;
	    double longitude, latitude;
	    // Read the first line of the archive
	    cin >> name
	    	>> numCustomers
	    	>> numStations
	    	>> maxTime
	    	>> maxDistance
	    	>> speed
	    	>> serviceTime
	    	>> refuelTime;
	    	
	 	// Read the contents of the archive
	    while(cin >> id) {
	    	cin >> ntype 
	    		>> longitude
	    		>> latitude;
	    	
	    	// Set up the nodes
	    	Node node;
	    	node.setNodeData(id, ntype, longitude, latitude);

	    	if(ntype == 'c') {
	    		customerNodes.push_back(node);
	    	}
	    	else if(ntype == 'f') {
	    		fuelNodes.push_back(node);
	    	}
	    	else if(ntype == 'd') {
	    		depot.setNodeData(id, ntype, longitude, latitude);
	    	}
	    }
	    cin.rdbuf(cinbuf);
	    in.close();
	}
	else {
		cout << "Error opening the dataset!\n";
		return -1;
	}
	return 0;
}