#include <iostream>
#include <string>
#include "instance_util.h"
using namespace std;

int main(int argc, char const *argv[]) {
	if(argc < 2) {
		cout << "Error: Expected more parameters." << '\n';
		//cout << "Follow the format: ./GVRPGreedyTaboo instanceName debug_mode" << '\n';
		//cout << "Choose 1 to enter the debug mode. Otherwise, type 0." << '\n';
		//cout << "Try: ./GVRPGreedyTaboo AB101 1" << '\n';
		return 0;
	}

    cout.setf(ios::fixed);
    cout.precision(10);

    //bool debug_mode = (bool) stoi(argv[2]);

	Instance gvrp(argv[1]);
	gvrp.loadData();
	return 0;
}