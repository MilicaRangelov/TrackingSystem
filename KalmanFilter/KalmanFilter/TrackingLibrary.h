#pragma once

#include "ObjectTracker.h"
#include <vector>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

#ifdef TRACKINGLIBRARY_EXPORTS
#define TRACKINGLIBRARY_API __declspec(dllexport)
#else
#define TRACKINGLIBRARY_API __declspec(dllimport)
#endif

extern "C"
{
	TRACKINGLIBRARY_API void* object_instance();

	TRACKINGLIBRARY_API int tracking_update(void* object, double* detections,  int num_of_objects, double* array_to_fill, int array_to_fill_size);

	TRACKINGLIBRARY_API void delete_object(void* object);
}