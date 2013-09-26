#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
#include <limits>
#include <cfloat>
using namespace std;
vector<float> data_x, data_y, data_z;

int slices_x = 100;
int slices_y = 1;
int slices_z = 100;

int main() {
	int line = 0;

	ifstream ifile("file_final.txt");
	string temp;
	float pos_x, pos_y, pos_z;
	while (ifile.fail() == false) {
		getline(ifile, temp);
		if (line > 7) {
			std::replace(temp.begin(), temp.end(), ',', ' ');

			stringstream ss(temp);

			ss >> pos_x >> pos_y >> pos_z;

			//cout << pos_x << " " << pos_y << " " << pos_z << " " << endl;
			data_x.push_back(pos_x);
			data_y.push_back(pos_y);
			data_z.push_back(pos_z);
		}
		line = line + 1;

	}

	int number_of_points = data_x.size();

	float min_x = FLT_MAX, max_x = -FLT_MAX;
	float min_y = FLT_MAX, max_y = -FLT_MAX;
	float min_z = FLT_MAX, max_z = -FLT_MAX;

	for (int i = 0; i < number_of_points; i++) {
		min_x = min(data_x[i], min_x);
		min_y = min(data_y[i], min_y);
		min_z = min(data_z[i], min_z);

		max_x = max(data_x[i], max_x);
		max_y = max(data_y[i], max_y);
		max_z = max(data_z[i], max_z);
	}
cout<<max_x<<" "<<max_y<<" "<<max_z<<endl;;

	float origin_x = min_x;
	float origin_y = min_y;
	float origin_z = min_z;

	//normalize data
	for (int i = 0; i < number_of_points; i++) {
		data_x[i] = data_x[i] - origin_x;
		data_y[i] = data_y[i] - origin_y;
		data_z[i] = data_z[i] - origin_z;
	}
	max_x = max_x - origin_x;
	max_y = max_y - origin_y;
	max_z = max_z - origin_z;

	min_x = min_y = min_z = 0;

	float bins_x = (fabs(max_x));
	float bins_y = (fabs(max_y));
	float bins_z = (fabs(max_z));

	bins_x = bins_x / float(slices_x);
	bins_y = bins_y / float(slices_y);
	bins_z = bins_z / float(slices_z);

	vector<float> bins((slices_x + 1) * (slices_y + 1) * (slices_z + 1));
	fill(bins.begin(), bins.end(), -FLT_MAX);
	for (int i = 0; i < number_of_points; i++) {
		int temp_x = floor(data_x[i] / bins_x);
		int temp_y = floor(data_y[i] / bins_y);
		int temp_z = floor(data_z[i] / bins_z);

		float value = bins[temp_x + temp_y * slices_y + temp_z * slices_y * slices_z];
		bins[temp_x + temp_y * slices_y + temp_z * slices_y * slices_z] = max(value, data_y[i]);
	}

	ofstream ofile("blah.txt");

	for (int i = 0; i < slices_x; i++) {
		for (int k = 0; k < slices_z; k++) {
			ofile << i << " " << k << " " << bins[i + 0 * slices_y + k * slices_y * slices_z] << endl;
		}
	}

	return 0;
}
