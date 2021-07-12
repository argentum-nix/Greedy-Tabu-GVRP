#ifndef INSTANCE_UTIL_H
#define INSTANCE_UTIL_H

#include <iostream>
#include <map>
#include <vector>
#include <string>

#define TARGET_DIRECTORY "./instances/"
#define DEBUG(x) cout << #x << " = " << x << endl

class Node {
public:
	int nodeID;				// id for solution route
	double longitude;  
	double latitude;
	char nodeType;   // d=depot, f=refueling stat, c=customer   	
	Node(int id, char t, double lon, double lat);
	~Node();
};


typedef std::pair<char, int> Key;
typedef std::map<Key, Node> Dict;
typedef std::pair<Key, int> Edge;

class Graph {
public:
	std::map<Key,std::vector<Edge>> adjList; // graph as map (C,0) -> [list of neighbours]
	int size;
	void printGraph();
	//void addEdge(Edge e1, Edge e2);
	void showAdjList();
	Graph();
	~Graph();		   		// destructor
};

class Instance {
public:
	std::string name;  		// instance name
	std::string dir;		// instance directory
	int maxTime;	   		// time limit
	int maxDistance;   		// distance limit
	int numCustomers;  
	int numStations; 
	int refuelTime;     
	int serviceTime;               
	double speed;      		// travelling speed

	Dict nodeMap;			// map with nodes, hashed by id and type
	Graph graph;			// adjacency list graph
	void loadData();
	Instance(std::string iName);
	~Instance();		   		// destructor
};


#endif