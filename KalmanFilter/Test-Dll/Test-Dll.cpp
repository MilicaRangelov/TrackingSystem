#include "TrackingLibrary.h"
#include <sstream>
#include <fstream>

vector<string> split(const string& s, char delim) {
	vector<string> result;
	stringstream ss(s);
	string item;
	while (getline(ss, item, delim)) {
		result.push_back(item);
	}
	return result;
}

int main() {

	void* tracker = object_instance();
	vector<MatrixXd> detect;
	MatrixXd pom;
	pom.setZero(1, 5);

	fstream detections;
	fstream traks;
	int counter = 0;

	detections.open("detections.txt", ios::in);
	traks.open("tracks.txt", ios::out | ios::app);

	if (detections.is_open()) {
		string video = "";
		getline(detections, video);

		traks << video << endl;
		string value = "";

		while (getline(detections, value)) {

			vector<string> values = split(value, ' ');

			if (values.size() > 2) {

				for (int i = 0; i < values.size(); i++) {
					double k = stod(values[i]);
					pom(0, i) = k;
				}
				cout << pom << endl;

				detect.push_back(pom);
			}
			else {

				traks << value << endl;
				counter++;

				if (counter < 2) {

					continue;
				}
				else {
					int num_of_objects;
					//num_of_objects = tracking_update(tracker, detect);
					detect.clear();
				

					/*for (int i = 0; i < numb_of_objects; i++) {
						for (int j = 0; j < 5; j++) {
							traks << track_update[i][j] << " ";
						}
						traks << endl;
					}*/

				}
				
			}

		}

	}
	detections.close();
	traks.close();
	delete_object(tracker);
	return 0;

}