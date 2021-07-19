#include <iostream>
#include <string>
#include <sys/time.h>
#include "instance_util.h"
#include "solver.h"
using namespace std;

int main(int argc, char const *argv[]) {
	struct timeval start, finish;
	if(argc < 2) {
		cout << "Error: Expected more parameters." << '\n';
		//cout << "Follow the format: ./GVRPGreedyTabu instanceName debug_mode" << '\n';
		//cout << "Choose 1 to enter the debug mode. Otherwise, type 0." << '\n';
		//cout << "Try: ./GVRPGreedyTabu AB101 1" << '\n';
		return 0;
	}

    cout.setf(ios::fixed);
    cout.precision(10);

    //bool debug_mode = (bool) stoi(argv[2]);

	Instance gvrp(argv[1]);
	
	int archiveStatus = gvrp.loadData();
	if(archiveStatus == -1) {
		// Archive was not found!
		return 0;
	}

	gettimeofday(&start, 0);
	GVRPSolver solve(&gvrp);
	gettimeofday(&finish, 0);

	double execTime = finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec) * 1e-6;
	cout << execTime << " [s]" << endl;
	return 0;
}