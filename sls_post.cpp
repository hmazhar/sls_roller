#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
#include <limits>
#include <cfloat>
#include <iomanip>
#ifdef __APPLE__
#include <GL/glew.h>
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif
using namespace std;

#include "xyz_io.hpp"

void ReadData(string filename) {
	int point_num;
	xyz_header_read(filename.c_str(), &point_num);
	double *xyz;
	xyz = new double[3 * point_num];
	xyz_data_read(filename.c_str(), point_num, xyz);
}

int main() {

	cout << "\n";
	cout << "  XYZ_DATA_READ has read the data.\n";

	cout << "\n";
	cout << "  Sample data:\n";
	cout << "\n";

	for (int k = 1; k <= 11; k++) {
		//int i = ((11 - k) * 1 + (k - 1) * point_num) / (11 - 1) - 1;
		//cout << "  " << setw(4) << i << "  " << setw(12) << xyz[0 + i * 3] << "  " << setw(12) << xyz[1 + i * 3] << "  " << setw(12) << xyz[2 + i * 3] << "\n";
	}

	//delete[] xyz;

	return 0;
}
