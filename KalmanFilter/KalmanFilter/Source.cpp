#include "ObjectTracker.h"
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

	ObjectTracker* tracker = new ObjectTracker();

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
					tracker->Update(detect);
					detect.clear();
					vector<string> output = tracker->get_status();

					for (int i = 0; i < output.size(); i++) {
						traks << output[i] << endl;
					}

				}
			}

		}

	}
	detections.close();
	traks.close();
	delete tracker;
	

	return 0;

}