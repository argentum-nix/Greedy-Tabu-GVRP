#ifndef INSTANCE_UTIL_H
#define INSTANCE_UTIL_H

#include <iostream>
#include <map>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

#define TARGET_DIRECTORY "./instances/"
#define DEBUG(x) cout << #x << " = " << x << endl

class Node {
public:
	int nodeID;				// id for solution route
	double longitude;  		// node latitude
	double latitude;		// node longitude
	char nodeType;   	    // d=depot, f=AFS, c=customer   	
	Node();
	void setNodeData(int id, char t, double lon, double lat);
	~Node();
};

class Instance {
public:
	std::string name;  		// instance name
	std::string dir;		// instance directory
	int maxTime;	   		// time limit
	int maxDistance;   		// distance limit
	int numCustomers;  		// number of customs
	int numStations; 		// numbor of AFS
	int refuelTime;     	// time it takes to refuel
	int serviceTime;        // time it takes to serve a client
	double speed;      		// travelling speed

	std::vector<Node> customerNodes;
	std::vector<Node> fuelNodes;
	Node depot;

	int loadData();
	Instance(std::string iName);
	~Instance();
};


#endif