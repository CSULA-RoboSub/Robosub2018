#ifndef DVLHELPER_H_
#define DVLHELPER_H_

#include <cmath>
#include <algorithm>
// using namespace std;

// static const double PI = 3.1415927;
const double pi = 3.14159265358979323846;

class DVLHelper
{
	// private:
	public:
		DVLHelper();
		// ~DVLHelper();

		//positive is right negative is left
		static double getDirection(double x1, double y1, double x2, double y2, double yaw);
		static double getDistance(double x1, double y1, double x2, double y2);
		static double getPI(){ return pi; }
		static double rad2deg(double radTheta){ return (radTheta * (180.0/pi)); }
		static double deg2rad(double degTheta){ return (degTheta * (pi/180.0)); }
};

#endif  // DVLHELPER_H_
