#include <iostream>
#include <sys/time.h>
#include "instance_util.h"
#include "solver.h"
using namespace std;

int main(int argc, char const *argv[]) {
	if(argc < 2) {
		cout << "Error: Expected more parameters." << '\n';
		cout << "Follow the format: ./GVRPGreedyTabu instanceName" << '\n';
		cout << "Try: ./GVRPGreedyTabu AB101" << '\n';
		return 0;
	}

	/*TESTING*/
	int numIter = stoi(argv[2]);
	int lenTabu = stoi(argv[3]);


    cout.setf(ios::fixed);
    cout.precision(10);

    struct timeval start, finish;
    /* CHANGED FOR TESTING*/
	Instance gvrp(argv[1], numIter, lenTabu);
	
	int archiveStatus = gvrp.loadData();
	if(archiveStatus == -1) {
		// Archive was not found!
		return 0;
	}

	gettimeofday(&start, 0);
	GVRPSolver solve(&gvrp);
	gettimeofday(&finish, 0);
	
	double execTime = finish.tv_sec - start.tv_sec + (finish.tv_usec - start.tv_usec) * 1e-6;
	solve.setExecTime(execTime);
	solve.writeSolution();
	return 0;
}