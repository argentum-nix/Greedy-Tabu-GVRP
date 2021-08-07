#include <sstream>
#include "instance_util.h"
using namespace std;


Node::Node() {}

void Node::setNodeData(int id, char t, double lon, double lat) {
	nodeID = id;
	nodeType = t;
	longitude = lon;
	latitude = lat;
}

Node::~Node() {}

Instance::Instance(std::string iName, int i, int t) {
	dir = TARGET_DIRECTORY;
	name = iName;
	iter = i;
	lentabu = t;
}

Instance::~Instance() {}

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
	cout << "Guarde el total de " << customerNodes.size() << " clientes\n";
	cout << "Guarde el total de " << fuelNodes.size() << " estaciones\n";
	return 0;
}