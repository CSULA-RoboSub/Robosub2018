#include <iostream>

// Include this header file to get access to the EzAsyncData class.
#include "vn/ezasyncdata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

// ros 
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ez_async_data/Rotation.h"
//#include "vn/vector.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vn100");

	ros::NodeHandle nh;
	ros::Publisher ypr_pub = nh.advertise<ez_async_data::Rotation>("current_rotation", 0);
	//ros::Publisher y_pub = nh.advertise<std_msgs::Float32>("yaw", 10);
	//ros::Publisher p_pub = nh.advertise<std_msgs::Float32>("pitch", 10);
	//ros::Publisher r_pub = nh.advertise<std_msgs::Float32>("roll", 10);
	// This example walks through using the EzAsyncData class to easily access
	// asynchronous data from a VectorNav sensor at a slight performance hit which is
	// acceptable for many applications, especially simple data logging.

	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	// const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS1";                    // Linux format for physical serial port.
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	// const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
	// const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t SensorBaudrate = 115200;

	// We create and connect to a sensor by the call below.
	EzAsyncData* ez = EzAsyncData::connect(SensorPort, SensorBaudrate);

	// Throughout this example, we have been using the ez->currentData() to get the most
	// up-to-date readings from the sensor that have been processed. When called, this
	// method returns immediately with the current values, thus the reason we have to
	// put the Thread::sleepMs(200) in the for loop. Otherwise, we would blaze through
	// the for loop and just print out the same values. The for loop below illustrates
	// this.

	cout << "current_rotation topic start" << endl;

	ros::Rate loop_rate(30); //for delay use
	ez_async_data::Rotation ypr;
	while(ros::ok())
	{
		CompositeData cd = ez->currentData();

		if (cd.hasYawPitchRoll()){
			//cout << "Current YPR: " << cd.yawPitchRoll() << endl;
			vec3f cd_ypr = cd.yawPitchRoll();
			ypr.yaw = cd_ypr[0];
			ypr.pitch = cd_ypr[1];
			ypr.roll = cd_ypr[2];

			ypr_pub.publish(ypr);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	ez->disconnect();

	delete ez;

	return 0;
}
