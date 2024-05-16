// TrackingLibrary.cpp : Defines the exported functions for the DLL.
#include "pch.h" 
#include <utility>
#include <limits.h>
#include "TrackingLibrary.h"
#define MAX_FRAME 27000

//extern "C"
//{
//	TRACKINGLIBRARY_API ObjectTrackerInterface* createTracker() { return new ObjectTracker(); }
//	TRACKINGLIBRARY_API void destroyTracker(ObjectTrackerInterface* tracker) { delete tracker; }
//}

static int frame = 0;

extern "C"
{
	void* object_instance()
	{
		ObjectTracker* object_tracker = new ObjectTracker ();
		void* object = reinterpret_cast<void*>(object_tracker);
		return  object;
	}

	int tracking_update(void* object, double* detections, int num_of_objects, double* array_to_fill, int array_to_fill_size)
	{
		ObjectTracker* tracker = reinterpret_cast<ObjectTracker*>(object);
		vector<MatrixXd> detc;
		int k = 0;
		for (int i = 0; i < (num_of_objects); i++) {
			MatrixXd pom;
			pom.setZero(1, 5);
			for (int j = 0; j < 5; j++) {
				pom(0, j) = detections[k++];	
			}
			detc.push_back(pom);
		}
		vector<vector<double>> trck = tracker->Update(detc);
		detc.clear();
		k = 0;
		frame++;
		if (frame == MAX_FRAME) {
			tracker->object_id = 1;
			frame = 0;
		}

		for (int i = 0; i < trck.size(); i++) {
			for (int j = 0; j < 5; j++) {
				if (k >= array_to_fill_size)
					return -1;
				array_to_fill[k++] = trck[i][j];
			}
		}
		return trck.size();
	}

	void delete_object(void* object)
	{
		ObjectTracker* tracker = reinterpret_cast<ObjectTracker*>(object);
		delete tracker;
	}

}