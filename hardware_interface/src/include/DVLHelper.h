#ifndef DVLHELPER_H_
#define DVLHELPER_H_

#include <cmath>
#include <algorithm>
// using namespace std;

static const float PI = 3.1415927;

class DVLHelper
{
	// private:
	public:
		DVLHelper();
		// ~DVLHelper();

		//positive is right negative is left
		static float getDirection(float x1, float y1, float x2, float y2, float yaw);
		static float getDistance(float x1, float y1, float x2, float y2);
		static float getPI(){ return PI; }
};

#endif  // DVLHELPER_H_
