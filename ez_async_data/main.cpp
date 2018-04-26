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

	// Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
//	for (int i = 0; i < 25; i++)
//	{
//		Thread::sleepMs(200);

		// This reads the latest data that has been processed by the EzAsyncData class.
//		CompositeData cd = ez->currentData();

		// Make sure that we have some yaw, pitch, roll data.
//		if (!cd.hasYawPitchRoll())
//			cout << "YPR Unavailable." << endl;
//		else
//			cout << "Current YPR: " << cd.yawPitchRoll() << endl;
//	}

	// Most of the asynchronous data handling is done by EzAsyncData but there are times
	// when we wish to configure the sensor directly while still having EzAsyncData do
	// most of the grunt work.This is easily accomplished and we show changing the ASCII
	// asynchronous data output type here.
//	try
//	{
//		ez->sensor()->writeAsyncDataOutputType(VNYPR);
//	}
//	catch (...)
//	{
//		cout << "Error setting async data output type." << endl;
//		return -1;
//	}

//	cout << "[New ASCII Async Output]" << endl;

	// We can now display yaw, pitch, roll data from the new ASCII asynchronous data type.
//	for (int i = 0; i < 25; i++)
//	{
//		Thread::sleepMs(200);
//
//		CompositeData cd = ez->currentData();
//
//		if (!cd.hasYawPitchRoll())
//			cout << "YPR Unavailable." << endl;
//		else
//			cout << "Current YPR: " << cd.yawPitchRoll() << endl;
//	}

	// The CompositeData structure contains some helper methods for getting data
	// into various formats. For example, although the sensor is configured to
	// output yaw, pitch, roll, our application might need it as a quaternion
	// value. However, if we query the quaternion field, we see that we don't
	// have any data.

//	cout << "HasQuaternion: " << ez->currentData().hasQuaternion() << endl;

	// Uncommenting the line below will cause an exception to be thrown since
	// quaternion data is not available.

	// cout << "Current Quaternion: " << ez->currentData().quaternion() << endl;

	// However, the CompositeData structure provides the anyAttitude field
	// which will perform the necessary conversions automatically.

//	cout << "[Quaternion from AnyAttitude]" << endl;
//
//	for (int i = 0; i < 25; i++)
//	{
//		Thread::sleepMs(200);
//
//		// This reads the latest data that has been processed by the EzAsyncData class.
//		CompositeData cd = ez->currentData();
//
		// Make sure that we have some attitude data.
//		if (!cd.hasAnyAttitude())
//			cout << "Attitude Unavailable." << endl;
//		else
//			cout << "Current Quaternion: " << cd.anyAttitude().quat() << endl;
//	}

	// Throughout this example, we have been using the ez->currentData() to get the most
	// up-to-date readings from the sensor that have been processed. When called, this
	// method returns immediately with the current values, thus the reason we have to
	// put the Thread::sleepMs(200) in the for loop. Otherwise, we would blaze through
	// the for loop and just print out the same values. The for loop below illustrates
	// this.

	cout << "current_rotation topic start" << endl;

	ros::Rate loop_rate(100); //for delay use
	//for (int i = 0; i < 1000000; i++)
//-------------------------------
	//std_msgs::Float32 yaw;
	//std_msgs::Float32 pitch;
	//std_msgs::Float32 roll;
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
//----------------------------

	// Often, we would like to get and process each packet received from the sensor.
	// This is not realistic with ez->currentData() since it is non-blocking and we
	// would also have to compare each CompositeData struture for changes in the data.
	// However, EzAsyncData also provides the getNextData() method which blocks until
	// a new data packet is available. The for loop below shows how to output each
	// data packet received from the sensor using getNextData().

	// cout << "[getNextData Method]" << endl;

	// for (int i = 0; i < 25; i++)
	// {
	// 	CompositeData cd = ez->getNextData();

	// 	if (!cd.hasYawPitchRoll())
	// 		cout << "YPR Unavailable." << endl;
	// 	else
	// 		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	// }

	ez->disconnect();

	delete ez;

	return 0;
}
