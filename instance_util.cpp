#include <fstream>
#include <sstream>
#include <algorithm>
#include "instance_util.h"
using namespace std;


Node::Node() {

}

void Node::setNodeData(int id, char t, double lon, double lat) {
	nodeID = id;
	nodeType = t;
	longitude = lon;
	latitude = lat;
}

Node::~Node() {
	//cout << "[LOG] Node destroyed\n";
}


Instance::Instance(std::string iName) {
	dir = TARGET_DIRECTORY;
	name = iName;
}

Instance::~Instance() {
	//cout << "[LOG] Instance destroyed\n";
}


int Instance::loadData() {
	ifstream in(dir + name + ".dat");
	if(in.is_open()) {
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

	    depot.setNodeData(id, ntype, longitude, latitude);

	    /*DEBUG(name);
	    DEBUG(numStations);
	    DEBUG(numStations);
	    DEBUG(maxTime);
	    DEBUG(maxDistance);
	    DEBUG(speed);
	    DEBUG(serviceTime);
	    DEBUG(refuelTime);*/

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

	    	Node node;
	    	node.setNodeData(id, ntype, longitude, latitude);

	    	if(ntype == 'c') {
	    		customerNodes.push_back(node);
	    	}
	    	else if(ntype == 'f') {
	    		fuelNodes.push_back(node);
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