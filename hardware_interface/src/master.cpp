/*-------------------------------------------------------------------------------------------
This is a quick translation of the old arduino code from level_with_motors.ino to c++
hardware_interface is a much faster version of the arduino node.
Arduino cannot operate as a node that receives the amount of data we send

important note: dvl works with 0 to 359.999 degrees with 0 locked to north, 90 east,
                180 south, and 270 west.
                imu works with -179.999 to 179.999 degrees with 90 north, +- 180 east,
                -90 south, 0 west

todo: clean/split file up to make it more readable and modular
-------------------------------------------------------------------------------------------*/

#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <robosub/HControl.h>
#include <robosub/RControl.h>
#include <robosub/MControl.h>
#include <pathfinder_dvl/DVL.h>
#include <ez_async_data/Rotation.h>
#include <hardware_interface/MotorHorizontal.h>
#include <hardware_interface/MotorVertical.h>
#include "include/DVLHelper.h"

using namespace std;

//CHANGED PWM_Motors TO PWM_Motors_depth SINCE THERE ARE 2 DIFFERENT PWM CALCULATIONS
//ONE IS FOR DEPTH AND THE OTHER IS USED FOR MOTORS TO ROTATE TO PROPER NEW LOCATION
// int PWM_Motors_Depth;
// double dutyCycl_depth;
double assignedDepth;
double feetDepth_read;

//initializations for IMU
double pitch, yaw, roll, heading;
// bool firstIMUReading;

int i;
// int PWM_Motors_orient;
double rotatePowerMax = 120;
double abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
double ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
double q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
double eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
double temperature;
// double dutyCycl_orient;
double assignedYaw;

//Initialize ROS node
const double rotationUpperBound = 180;
const double rotationLowerBound = -180;
const double rotationMultiplier = 4.5;
const double rotationMinOffset = 35;
const double topDepth = 0.5;
const double bottomDepth = 12;
const double motorMax = 1700;
const double motorMin = 1300;
int mControlDirection;
double mControlPower;
double rControlPower;
double mControlDistance;
double mControlRunningTime;
double mControlMode5Timer; //time variable for timed movement
double centerTimer;
double rotationTimer;
double rotationTime;
// double movementTimer;
double movementTime;
bool subIsReady;
bool isGoingUp;
bool isGoingDown;
bool isTurningRight;
bool isTurningLeft;
bool keepTurningRight;
bool keepTurningLeft;
//bool flags for control modes
bool rControlMode3;
bool rControlMode4;
bool mControlMode1;
bool mControlMode2;
bool mControlMode3;
bool mControlMode4;
bool mControlMode5;
//bool flags for infinite movement
bool keepMovingForward;
bool keepMovingRight;
bool keepMovingBackward;
bool keepMovingLeft;

//dvl position data is in meters---------------
//dvl position variables-----------------------
double positionXPrev = 0;
double positionYPrev = 0;

double positionX = 0;
double positionY = 0;
double positionZ = 0;
double velocityX = 0;
double velocityY = 0;
double velocityZ = 0;

//position keeping variables
double keepPositionX = 0;
double keepPositionY = 0;
const double keepPositionThreshold = 0.4; //meters
const double keepPositionMaxPowerForwardBackward = 80;
const double keepPositionMaxPowerRightLeft = 80;

std_msgs::Float32 currentDepth;
std_msgs::Float32 dvlHeading;
robosub::HControl hControlStatus;
robosub::RControl rControlStatus;
robosub::MControl mControlStatus;
//msgs used to communicate with Arduino
hardware_interface::MotorVertical mVertical;
hardware_interface::MotorHorizontal mHorizontal;
//msg container used to offset mHorizontal for position keeping
hardware_interface::MotorHorizontal mHorizontalOffset;
hardware_interface::MotorHorizontal mHorizontalTotal;


ros::Publisher hControlPublisher;     //int: state, double: depth
ros::Publisher rControlPublisher;   //int: state, double: rotation
ros::Publisher mControlPublisher;   //int: state, int: direction, double: distance
ros::Publisher currentDepthPublisher;           //double: depth
ros::Publisher mHorizontalPublisher;
ros::Publisher mVerticalPublisher;

ros::Subscriber currentDepthSubscriber;
ros::Subscriber hControlSubscriber;   //int: state, double: depth
ros::Subscriber rControlSubscriber; //int: state, double: rotation
ros::Subscriber mControlSubscriber;
ros::Subscriber rotationSubscriber;
ros::Subscriber dvlSubscriber;

//depth control variables
// int pwm_submerge = 200;
// int pwm_emerge = 150;
double hControlPower;
//thrusters off value does not change
const double base_thrust = 1500;

//time variables
double elapsedTime, timeCur, timePrev, loopTime, loopTimePrev;
const int loopInterval = 20;

//the variable error will store the difference between the real_value_angle form IMU and desired_angle of 0 degrees. 
// double PID_pitch, PID_roll, pwmThruster_2, pwmThruster_1, pwmThruster_3, pwmThruster_4, error_roll, prev_error_roll = 0,error_pitch, prev_error_pitch = 0;
double PID_pitch, PID_roll, PID_depth, pwmThruster_1, pwmThruster_2, pwmThruster_3, pwmThruster_4, 
error_roll, prev_error_roll=0, error_pitch, prev_error_pitch=0, error_depth, prev_error_depth=0;

//////////////PID_pitch variables////////////////////
double pid_p_pitch=0;
double pid_d_pitch=0;
double pid_i_pitch=0;
/////////////////PID_pitch constants/////////////////
const double kp_pitch=2;//11;//3.55;//3.55
const double kd_pitch=0.7;//0.75;//2.05;//2.05
const double ki_pitch=0.0003;//0.003
///////////////////////////////////////////////

//////////////PID_roll variables////////////////////
double pid_p_roll=0;
double pid_d_roll=0;
double pid_i_roll=0;
/////////////////PID_roll constants/////////////////
const double kp_roll=2;//11;//3.55;//3.55
const double kd_roll=0.7;//0.75;//2.05;//2.05
const double ki_roll=0.0003;//0.003
///////////////////////////////////////////////

//////////////PID_depth variables////////////////////
double pid_p_depth=0;
double pid_d_depth=0;
double pid_i_depth=0;
/////////////////PID_depth constants/////////////////
const double kp_depth=300;//11;//3.55;//3.55
const double kd_depth=0.75;//0.75;//2.05;//2.05
const double ki_depth=0.003;//0.003

//double thrust=base_thrust; //initial value of thrust to the thrusters
const double desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
//// threshold for going_up, going_down and hover
const double heightThreshold = 0.3;
///////////////////////////////////////////////

/////////////////PID_heading constants and variables/////////////////
const double kp_heading=11;//11;//3.55;//3.55
const double kd_heading=0.7;//0.75;//2.05;//2.05
const double ki_heading=0.003;

double pid_i_heading = 0;
// double headingTimePrev = 0;
double prev_error_heading = 0;
double rotationThreshold = 1;
///////////////////////////////////////////////

/////////////////PID_positionX constants and variables/////////////////
// const double kp_positionX=9;//11;//3.55;//3.55
const double kd_positionX=0.8;//0.75;//2.05;//2.05
const double ki_positionX=0.003;//0.003

double pid_i_positionX = 0;
double rightLeftDistancePrev = 0;
///////////////////////////////////////////////

/////////////////PID_positionY constants and variables/////////////////
// const double kp_positionY=9;//11;//3.55;//3.55
const double kd_positionY=0.8;//0.75;//2.05;//2.05
const double ki_positionY=0.003;//0.003

double pid_i_positionY = 0;
double forwardBackwardDistancePrev = 0;
///////////////////////////////////////////////

double positionControlMOffset = 0;

double degreeToTurn();
void rotateRightDynamically();
void rotateLeftDynamically();

//returns time in milliseconds

//----------------------------------------------------------------------------
//---------------------------- helper functions ------------------------------
//----------------------------------------------------------------------------

//returns current time in millis
double millis(){
  return ros::WallTime::now().toNSec()/1000000;
}

//sets position keeping to current position
void setKeepPosition(){
  keepPositionX = positionX;
  keepPositionY = positionY;
}

//sets offset for position keeping use
void setMHorizontalOffset(const double t5, const double t6, const double t7, const double t8){
  mHorizontalOffset.t5 = t5;
  mHorizontalOffset.t6 = t6;
  mHorizontalOffset.t7 = t7;
  mHorizontalOffset.t8 = t8;
}

//set vertical motor power
//pass in -1 to ignore that motor
void setMVertical(const double t1, const double t2, const double t3, const double t4){
  if(t1 > 0)
    mVertical.t1 = t1;
  if(t2 > 0)
    mVertical.t2 = t2;
  if(t3 > 0)
    mVertical.t3 = t3;
  if(t4 > 0)
    mVertical.t4 = t4;
}

//set horizontal motor power
//pass in -1 to ignore that motor
void setMHorizontal(const double t5, const double t6, const double t7, const double t8){
  if(t5 > 0)
    mHorizontal.t5 = t5;
  if(t6 > 0)
    mHorizontal.t6 = t6;
  if(t7 > 0)
    mHorizontal.t7 = t7;
  if(t8 > 0)
    mHorizontal.t8 = t8;
}

//publish motor power to ros
void publishMotors(){
  mHorizontalTotal.t5 = mHorizontal.t5 + mHorizontalOffset.t5;
  mHorizontalTotal.t6 = mHorizontal.t6 + mHorizontalOffset.t6;
  mHorizontalTotal.t7 = mHorizontal.t7 + mHorizontalOffset.t7;
  mHorizontalTotal.t8 = mHorizontal.t8 + mHorizontalOffset.t8;
  mVerticalPublisher.publish(mVertical);
  mHorizontalPublisher.publish(mHorizontalTotal);
}

//sets motors to publish base_thrust
void motorsOff(){
  setMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
  setMVertical(base_thrust,base_thrust,base_thrust,base_thrust);
  setMHorizontalOffset(0, 0, 0, 0);
}

//pid for forward axis
double positionForwardPID(double error_positionY){
  int pid_d_positionY = 0;
  if(-keepPositionThreshold < error_positionY && error_positionY < keepPositionThreshold)
    pid_i_positionY = pid_i_positionY+(ki_positionY*error_positionY);
  else
    pid_i_positionY = 0;

  pid_d_positionY = kd_positionY*((error_positionY - forwardBackwardDistancePrev)/elapsedTime);
  return pid_d_positionY + pid_i_positionY;
}

//pid for right axis
double positionRightPID(double error_positionX){
  int pid_d_positionX = 0;
  if(-keepPositionThreshold < error_positionX && error_positionX < keepPositionThreshold)
    pid_i_positionX = pid_i_positionX+(ki_positionX*error_positionX);
  else
    pid_i_positionX = 0;

  pid_d_positionX = kd_positionX*((error_positionX - rightLeftDistancePrev)/elapsedTime);
  return pid_d_positionX + pid_i_positionX;
}

//Return 0 to 180
double degreeToTurn(){
  double difference = max(yaw, assignedYaw) - min(yaw, assignedYaw);
  if (difference > rotationUpperBound) return 360-difference;
  else return difference;
}

double rotationPID(){
  double error_heading = degreeToTurn();
  // double rotationElapsedTime = (loopTime - headingTimePrev)/base_thrust;
  // headingTimePrev = loopTime;

  double pid_p_heading = kp_heading*error_heading;
  double pid_d_heading = kd_heading*((error_heading - prev_error_heading)/elapsedTime);

  if(-3 < error_heading && error_heading < 3)
    pid_i_heading = pid_i_heading+(ki_heading*error_heading);
  else
    pid_i_heading = 0;

  return pid_p_heading + pid_i_heading + pid_d_heading;
}

//dynamic rotations left/right
void rotateLeftDynamically(){
  // double rotatePower = PWM_Motors_orient * rotationMultiplier + rotationMinOffset;
  double rotatePower = rotationPID();

  if(rotatePower > rControlPower && isTurningLeft) rotatePower = rControlPower;
  if(rotatePower > rotatePowerMax) rotatePower = rotatePowerMax;
  if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
    setMHorizontal(-1, base_thrust + rotatePower, -1, base_thrust + rotatePower);
  }
  else{
    setMHorizontal(base_thrust - rotatePower, -1, base_thrust + rotatePower, -1);
  }
}

void rotateRightDynamically(){
  // double rotatePower = PWM_Motors_orient * rotationMultiplier + rotationMinOffset;
  double rotatePower = rotationPID();

  if(rotatePower > rControlPower && isTurningRight) rotatePower = rControlPower;
  if(rotatePower > rotatePowerMax) rotatePower = rotatePowerMax;
  if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
    setMHorizontal(-1, base_thrust - rotatePower, -1, base_thrust - rotatePower);
  }
  else{
    setMHorizontal(base_thrust + rotatePower, -1, base_thrust - rotatePower, -1);
  }
}

//publish finished movement
void movementControlFinish(){
  setMHorizontal(base_thrust, base_thrust, base_thrust, base_thrust);
  centerTimer = 0;
  // movementTimer = 0;
  mControlMode5Timer = 0;
  mControlDirection = 0;
  mControlRunningTime = 0;
  mControlPower = 0;
  mControlStatus.state = 0;
  mControlStatus.mDirection = 0;
  mControlStatus.power = 0;
  mControlStatus.distance = mControlDistance;
  mControlStatus.runningTime = 0;
  mControlDistance = 0;
  mControlPublisher.publish(mControlStatus);

  setKeepPosition();
}

//-----------------------------------------------------------------------
//------------------------ ros callbacks --------------------------------
//-----------------------------------------------------------------------

void rotationCallback(const ez_async_data::Rotation& rotation){
  yaw = rotation.yaw;
  roll = -rotation.roll;
  pitch = -rotation.pitch;
}

//xvel is left right strafe velocity, yvel is forward backward velocity,
//zvel is up down velocity
void dvlCallback(const pathfinder_dvl::DVL& dvl_status){
  positionX = dvl_status.xpos;
  positionY = dvl_status.ypos;
  positionZ = dvl_status.zpos;

  velocityX = dvl_status.xvel;
  velocityY = dvl_status.yvel;
  velocityZ = dvl_status.zvel;
}

void currentDepthCallback(const std_msgs::Float32& currentDepth){
  feetDepth_read = currentDepth.data;
}

void hControlCallback(const robosub::HControl& hControl) {
  int hState = hControl.state;
  double depth = hControl.depth;
  hControlPower = hControl.power;

  if(hControl.state == 0){
    if(!isGoingUp && !isGoingDown){
      isGoingDown = true;
      ROS_INFO("Going down...");
      // ROS_INFO(depthChar);
      // ROS_INFO("ft...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still running. Command abort.");

    if(depth == -1 || depth + assignedDepth >= bottomDepth)
      assignedDepth = bottomDepth;
    else
      assignedDepth = assignedDepth + depth;
  }
  else if(hControl.state == 1){
    if(isGoingUp || isGoingDown){
      isGoingUp = false;
      isGoingDown = false;
      ROS_INFO("Height control is now cancelled\n");
      assignedDepth = feetDepth_read + heightThreshold;
    }
  }
  else if(hControl.state == 2){
    if(!isGoingUp && !isGoingDown){
      isGoingUp = true;
      ROS_INFO("Going up...");
      // ROS_INFO(depthChar);
      // ROS_INFO("ft...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still running. Command abort.");
      
    if(depth == -1 || depth >= assignedDepth - topDepth)
      assignedDepth = topDepth;
    else
      assignedDepth = assignedDepth - depth;
  }
  else if(hControl.state == 4){
    assignedYaw = yaw;
    assignedDepth = topDepth;
    setKeepPosition();
    if(!subIsReady){
      ROS_INFO("Motors unlocked.");
    }
    subIsReady = true;
  }
  else if(hControl.state == 5){
    if(subIsReady){
      ROS_INFO("Motors are locked.");
    }
    assignedDepth = topDepth;
    subIsReady = false;
    setMVertical(base_thrust,base_thrust,base_thrust,base_thrust);
    setMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
  }

  hControlStatus.state = hState;
  hControlStatus.depth = assignedDepth;
  hControlStatus.power = hControlPower;
  
  hControlPublisher.publish(hControlStatus);

}


void rControlCallback(const robosub::RControl& rControl){
  if(rControl.rotation > 180){
    ROS_INFO("Error rotation must be less than or equal to 180 degrees");
    return;
  }

  double rotation = rControl.rotation;
  rControlPower = rControl.power;
  rControlStatus.state = rControl.state;
  rControlStatus.rotation = rControl.rotation;
  rControlStatus.power = rControlPower;

  if(rControl.state == 0){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      if(rotation == -1) {
        keepTurningLeft = true;
        //Testing---------------------------------------------------
        rotationTimer = 0;
      }
      else{
        if (yaw - rotation < rotationLowerBound)
          assignedYaw = yaw - rotation + 360;
        else
          assignedYaw = yaw - rotation;
      }
      isTurningLeft = true;
      ROS_INFO("Turning left...");
      // ROS_INFO(rotationChar);
      // ROS_INFO("degree...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still rotating. Command abort.");
  }
  else if(rControl.state == 1){
    if(isTurningRight || isTurningLeft || keepTurningRight || keepTurningLeft || rControlMode3 || rControlMode4){
      ROS_INFO("Rotation control is now cancelled\n");
      rControlStatus.state = 1;
      rControlStatus.rotation = 0;
      rControlStatus.power = rControlPower;
      rControlPublisher.publish(rControlStatus);
      assignedYaw = yaw;
    }
    isTurningRight = false;
    isTurningLeft = false;
    keepTurningRight = false;
    keepTurningLeft = false;
    rControlMode3 = false;
    rControlMode4 = false;
  }
  else if(rControl.state == 2){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      if(rotation == -1) {
        keepTurningRight = true;
        //Testing---------------------------------------------------------------
        rotationTimer = 0;
      }
      else{
        if (yaw + rotation > rotationUpperBound)
          assignedYaw = yaw + rotation - 360;
        else
          assignedYaw = yaw + rotation;
      }
      isTurningRight = true;
      ROS_INFO("Turning right...");
      // ROS_INFO(rotationChar);
      // ROS_INFO("degree...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still rotating.Command abort.");
  }
  else if(rControl.state == 3){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      rControlMode3 = true;
      rotationTime = 3;
      rotationTimer = 0;
      ROS_INFO("Rotate with front camera horizontal distance.\n");
    }
    else
      ROS_INFO("Sub is still rotating.Command abort.");
  }
  else if(rControl.state == 4){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      rControlMode4 = true;
      ROS_INFO("Keep rotating with front camera horizontal distance.\n");
    }
    else
      ROS_INFO("Sub is still rotating.Command abort.");
    
  }
}

// MControl
// state => 0: off, 1: on with power, 2: on with distance, 3: centered with front cam, 4: centered with bottom cam
// direction => 1: forward, 2: right, 3: backward, 4: left
// power => 0: none, x: x power add to the motors
// distance => 0: none, x: x units away from the object
void mControlCallback(const robosub::MControl& mControl){
  double power = mControl.power;
  double distance = mControl.distance;
  double mode5Time = mControl.runningTime;

  string directionStr;
  mControlStatus.state = mControl.state;
  mControlStatus.mDirection = mControl.mDirection;
  mControlStatus.power = mControl.power;
  mControlStatus.distance = mControl.distance;
  mControlStatus.runningTime = mControl.runningTime;


  if(mControl.state == 0){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5){
      // T6.writeMicroseconds(base_thrust);
      // T8.writeMicroseconds(base_thrust);
      // T5.writeMicroseconds(base_thrust);
      // T7.writeMicroseconds(base_thrust);
      ROS_INFO("Movement control is now cancelled\n");
      mControlPublisher.publish(mControlStatus);
      positionControlMOffset = 0;
    }
    mControlDirection = 0;
    mControlPower = 0;
    mControlDistance = 0;
    setMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
    mControlMode1 = false;
    mControlMode2 = false;
    mControlMode3 = false;
    mControlMode4 = false;
    mControlMode5 = false;
    keepMovingForward = false;
    keepMovingRight = false;
    keepMovingBackward = false;
    keepMovingLeft = false;
    setKeepPosition();
  }
  else if(mControl.state == 1){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else if(mControl.mDirection != 1 && mControl.mDirection != 2 && mControl.mDirection != 3 && mControl.mDirection != 4)
      ROS_INFO("Invalid direction with state 1. Please check the program and try again.\n");
    else if(mControl.power == 0)
      ROS_INFO("Invalid power with state 1. Please check the program and try again.");
    else{
      if(mControl.mDirection == 1){
        keepMovingForward = true;
        directionStr = "forward";
      }
      else if(mControl.mDirection == 2){
        keepMovingRight = true;
        directionStr = "right";
      }
      else if(mControl.mDirection == 3){
        keepMovingBackward = true;
        directionStr = "backward";
      }
      else if(mControl.mDirection == 4){
        keepMovingLeft = true;
        directionStr = "left";
      }

      directionStr = "Moving " + directionStr + " with power...";
      ROS_INFO("%s", directionStr.c_str());
      // ROS_INFO(powerChar);
      // ROS_INFO("...\n");

      //Testing -----------------------------------------------------------
      // movementTimer = 0;
      mControlMode1 = true;
      mControlDirection = mControl.mDirection;
      mControlPower = mControl.power;
      mControlDistance = 0;
    }
  }
  else if(mControl.state == 2){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else if(mControl.mDirection != 1 && mControl.mDirection != 2 && mControl.mDirection != 3 && mControl.mDirection != 4)
      ROS_INFO("Invalid direction with state 2. Please check the program and try again.\n");
    else if(mControl.power == 0)
      ROS_INFO("Invalid power with state 2. Please check the program and try again.");
    // else if(mControl.mDirection != 1)
    //   ROS_INFO("Invalid direction with state 2. Please check the program and try again.\n");
    else{
      if(mControl.mDirection == 1){
        directionStr = "forward";
      }
      else if(mControl.mDirection == 2){
        directionStr = "right";
      }
      else if(mControl.mDirection == 3){
        directionStr = "backward";
      }
      else if(mControl.mDirection == 4){
        directionStr = "left";
      }
      // ROS_INFO("Adjusting distance to...");
      // ROS_INFO(distanceChar);
      // ROS_INFO("away from the target.../n");
      mControlMode2 = true;
      mControlPower = mControl.power;
      mControlDirection = mControl.mDirection;
      mControlDistance = mControl.distance;
      //save starting point for use in distance checking
      positionXPrev = positionX;
      positionYPrev = positionY;
      cout << "going " << directionStr << " " << mControlDistance << " meters."<< endl;
    }
  }
  else if(mControl.state == 5){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else if(mControl.mDirection != 1 && mControl.mDirection != 2 && mControl.mDirection != 3 && mControl.mDirection != 4)
      ROS_INFO("Invalid direction with state 5. Please check the program and try again.\n");
    else if(mControl.power == 0)
      ROS_INFO("Invalid power with state 5. Please check the program and try again.");
    else{
      if(mControl.mDirection == 1){
        directionStr = "forward";
      }
      else if(mControl.mDirection == 2){
        directionStr = "right";
      }
      else if(mControl.mDirection == 3){
        directionStr = "backward";
      }
      else if(mControl.mDirection == 4){
        directionStr = "left";
      }

      directionStr = "Moving " + directionStr + " with time...";
      ROS_INFO("%s", directionStr.c_str());
      // ROS_INFO(powerChar);
      // ROS_INFO("for...");
      // ROS_INFO(timeChar);
      // ROS_INFO("seconds...\n");

      mControlMode5 = true;
      mControlDirection = mControl.mDirection;
      mControlPower = mControl.power;
      mControlRunningTime = mControl.runningTime;
      mControlMode5Timer = loopTime;
      mControlDistance = 0;
    }
  }

}


//----------------------------------------------------------------------------
//------------------------------ control functions ---------------------------
//----------------------------------------------------------------------------

//depth and stabilization control function
//roll left is negative roll right is positive (roll currently inverted of this)
//pitch backward is positive pitch forward is negative
void heightControl(){
  if(!subIsReady){ return; }

  // timePrev = timeCur;  // the previous time is stored before the actual time read
  // timeCur = millis();  // actual time read
  // elapsedTime = (loopTime - timePrev) /base_thrust;      //base_thrust; 
  // timePrev = loopTime; //update to current time after done with previous time
  /*///////////////////////////P I Ds///////////////////////////////////*/
  
  error_pitch = pitch - desired_angle; 
  error_roll = roll - desired_angle;
  error_depth = feetDepth_read - assignedDepth;
  
  pid_p_pitch = kp_pitch*error_pitch;
  pid_p_roll = kp_roll*error_roll;
  pid_p_depth = kp_depth*error_depth;
  
  if(-1 < error_pitch && error_pitch < 1){
    pid_i_pitch = pid_i_pitch+(ki_pitch*error_pitch);  
  }
  
  if(-1 < error_roll && error_roll < 1){
    pid_i_roll = pid_i_roll+(ki_roll*error_roll);
  } 

  if(-heightThreshold < error_depth && error_depth < heightThreshold)
  {
    pid_i_depth = pid_i_depth+(ki_depth*error_depth);
  } else {
    pid_i_depth = 0;
  }
  
  pid_d_pitch = kd_pitch*((error_pitch - prev_error_pitch)/elapsedTime);
  pid_d_roll = kd_roll*((error_roll - prev_error_roll)/elapsedTime);
  pid_d_depth = kd_depth*((error_depth - prev_error_depth)/elapsedTime);
  
  PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  PID_depth = pid_p_depth + pid_i_depth + pid_d_depth;

  //max pid adjustment is full 800 which is the range of motor power
  if(PID_pitch < -800){PID_pitch=-800;}
  if(PID_pitch > 800){PID_pitch=800;}
  if(PID_roll < -800){PID_roll=-800;}
  if(PID_roll > 800){PID_roll=800;}

  //depth max speed set to pwm_submerge and pwm_emerge
  //negative number for submerge
  if(PID_depth < -hControlPower){PID_depth = -hControlPower;}
  //positive for emerge
  if(PID_depth > hControlPower){PID_depth = hControlPower;}
  
  //Emerging and submerging thruster pwm requirments:
  /*Submerging: pwmThruster_1 > base_thrust 
                pwmThruster_2 < base_thrust
                pwmThruster_3 > base_thrust 
                pwmThruster_4 < base_thrust

    Emerging:   pwmThruster_1 < base_thrust 
                pwmThruster_2 > base_thrust
                pwmThruster_3 < base_thrust 
                pwmThruster_4 > base_thrust         
  */

  //emerging                                           
  // if (assignedDepth < (feetDepth_read - heightThreshold )){
  //   pwmThruster_1 = base_thrust - PID_pitch - PID_roll - PID_depth;
  //   pwmThruster_2 = base_thrust + PID_pitch - PID_roll + PID_depth;
  //   pwmThruster_3 = base_thrust - PID_pitch + PID_roll - PID_depth;
  //   pwmThruster_4 = base_thrust + PID_pitch + PID_roll + PID_depth;
  // }

  // //submerging
  // else if (assignedDepth > (feetDepth_read + heightThreshold)){
  //   pwmThruster_1 = base_thrust - PID_pitch - PID_roll - PID_depth;
  //   pwmThruster_2 = base_thrust + PID_pitch - PID_roll + PID_depth;
  //   pwmThruster_3 = base_thrust - PID_pitch + PID_roll - PID_depth;
  //   pwmThruster_4 = base_thrust + PID_pitch + PID_roll + PID_depth;
  // }
  
  //////Stabilization sum
  else{
    if(isGoingUp || isGoingDown){
      isGoingUp = false;
      isGoingDown = false;
      ROS_INFO("Assigned depth reached.\n");
      hControlStatus.state = 1;
      hControlStatus.depth = feetDepth_read;
      hControlStatus.power = hControlPower;
      hControlPublisher.publish(hControlStatus);
    }
    // pwmThruster_1 = base_thrust - PID_pitch - PID_roll - pid_i_depth;
    // pwmThruster_2 = base_thrust + PID_pitch - PID_roll + pid_i_depth;
    // pwmThruster_3 = base_thrust - PID_pitch + PID_roll - pid_i_depth;
    // pwmThruster_4 = base_thrust + PID_pitch + PID_roll + pid_i_depth;
  }
  
  pwmThruster_1 = base_thrust - PID_pitch - PID_roll - PID_depth;
  pwmThruster_2 = base_thrust + PID_pitch - PID_roll + PID_depth;
  pwmThruster_3 = base_thrust - PID_pitch + PID_roll - PID_depth;
  pwmThruster_4 = base_thrust + PID_pitch + PID_roll + PID_depth;

  ///////////Thruster power buffer//////////////
  //Thruster_1
  if(pwmThruster_1 < motorMin){pwmThruster_1= motorMin;}
  if(pwmThruster_1 > motorMax){pwmThruster_1=motorMax;}
  //Thruster_2
  if(pwmThruster_2 < motorMin){pwmThruster_2= motorMin;}
  if(pwmThruster_2 > motorMax){pwmThruster_2=motorMax;}
  //Thruster_3
  if(pwmThruster_3 < motorMin){pwmThruster_3= motorMin;}
  if(pwmThruster_3 > motorMax){pwmThruster_3=motorMax;}
  //Thruster_4
  if(pwmThruster_4 < motorMin){pwmThruster_4= motorMin;}
  if(pwmThruster_4 > motorMax){pwmThruster_4=motorMax;}
          
  prev_error_pitch = error_pitch;
  prev_error_roll = error_roll;
  prev_error_depth = error_depth;

  /*Create the PWM pulses with the calculated width for each pulse*/
  setMVertical(pwmThruster_1, pwmThruster_2, pwmThruster_3, pwmThruster_4);
}

//read rotation is from -179.999 to 179.999
//handles rotation and heading keeping
void rotationControl(){

  double delta = degreeToTurn();
  int fixedPower = rControlPower;
  if(fixedPower > rotatePowerMax) fixedPower = rotatePowerMax;

  if(keepTurningLeft){
    // //Turn on left rotation motor with fixed power
    if( ((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
      setMHorizontal(-1, base_thrust + fixedPower, -1, base_thrust + fixedPower);
    }
    else{
      setMHorizontal(base_thrust - fixedPower, -1, base_thrust + fixedPower, -1);
    }
    assignedYaw = yaw;

  }
  else if(keepTurningRight){
    //Turn on right rotation motor with fixed power
    if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
      setMHorizontal(-1, base_thrust - fixedPower, -1, base_thrust - fixedPower);
    }
    else{
      setMHorizontal(base_thrust + fixedPower, -1, base_thrust - fixedPower, -1);
    }
    assignedYaw = yaw;
    //Testing----------------------------
//    rotationTimer += 0.01;
//    if(rotationTimer > rotationTime)
//      keepTurningRight = false;
//    yaw -= 0.05;
//    if(yaw < rotationLowerBound)
//      yaw +=360;
  }
  // AutoRotation to the assignedYaw
  // else if(delta > rotationThreshold){
  else{
    // cout << "in rotationThreshold delta: " << delta << " assignedYaw: " << assignedYaw << " yaw: " << yaw << endl;
    // if(isTurningRight){
    //   // ROS_INFO("isTurningRight");
    //   // cout << "isTurningRight" << endl;
    //   rotateRightDynamically();
    // }
    // else if(isTurningLeft){
    //   // ROS_INFO("isTurningLeft");
    //   // cout << "isTurningLeft" << endl;
    //   rotateLeftDynamically();
    // }
    // double deltaRange = 4;
    if(yaw + delta > rotationUpperBound && (yaw + -assignedYaw) > 180){
      // ROS_INFO("yaw + delta > rotationUpperBound");
      // if((assignedYaw-deltaRange <= yaw - delta && yaw - delta <= assignedYaw+deltaRange) && isTurningLeft){
      if(isTurningLeft){
        // ROS_INFO("aw - delta == assignedYaw");
        // cout << "aw - delta == assignedYaw" << endl;
        rotateLeftDynamically();
      }else{
        // ROS_INFO("aw - delta == assignedYaw else");
        // cout << "aw - delta == assignedYaw else" << endl;
        rotateRightDynamically();
      }
    }
    else if(yaw - delta < rotationLowerBound && (-yaw + assignedYaw) > 180){
      // ROS_INFO("yaw - delta < rotationUpperBound");

      // cout << "yaw - delta < rotationUpperBound" << endl;
      // if((assignedYaw-deltaRange <= yaw + delta && yaw + delta <= assignedYaw+deltaRange) && isTurningRight){
      if(isTurningRight){
        // ROS_INFO("yaw + delta == assignedYaw");
        // cout << "yaw + delta == assignedYaw" << endl;
        rotateRightDynamically();
      }
      else{
        // ROS_INFO("yaw + delta == assignedYaw else");
        // cout << "yaw + delta == assignedYaw else" << endl;
        rotateLeftDynamically();
      }
    }
    else if(yaw < assignedYaw){
      // ROS_INFO("yaw < assignedYaw");
      // cout << "yaw < assignedYaw" << endl;
      rotateRightDynamically();
    }
    else if(yaw > assignedYaw){
      // ROS_INFO("yaw < assignedYaw else");
      // cout << "yaw < assignedYaw else" << endl;
      rotateLeftDynamically();
    }
    else{
      if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
        // cout << "base_thrust 6 and 8" << endl;
        setMHorizontal(-1, base_thrust, -1, base_thrust);
      }
      else{
        // cout << "base_thrust 5 and 7" << endl;
        setMHorizontal(base_thrust, -1, base_thrust, -1);
      }
    }
  }

  //finished rotation
  if(!keepTurningRight && !keepTurningLeft && delta <= rotationThreshold){
    // cout << "in no rotation" << endl;
    if(isTurningRight || isTurningLeft){
      isTurningRight = false;
      isTurningLeft = false;
      ROS_INFO("Assigned rotation reached.\n");
      rControlStatus.state = 1;
      rControlStatus.rotation = 0;
      rControlStatus.power = rControlPower;
      rControlPublisher.publish(rControlStatus);
    }
    
  }

}

//handles forward, backward, left strafe, and right strafe movment
void movementControl(){
  double mControlPowerTemp = mControlPower;
  if(mControlPowerTemp > 200) mControlPowerTemp = 200;

  if(mControlMode1){
    if(keepMovingForward){
      setMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
    }
    else if(keepMovingRight){
      setMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
    }
    else if(keepMovingBackward){
      setMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
    }
    else if(keepMovingLeft){
      setMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
    }
  }
  else if(mControlMode2){
    // cout << "in mControlMode2" << endl;
    if(mControlDirection == 1){
      // T6.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust - mControlPowerTemp);
      // cout << "in mControlMode2 direction 1" << endl;
      setMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
      
      // ROS_INFO("moving forward...");
    }
    //right
    else if(mControlDirection == 2){
      // T5.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust + mControlPowerTemp);
      setMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
      
      // ROS_INFO("moving right...");
    }
    //backward
    else if(mControlDirection == 3){
      // T6.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust + mControlPowerTemp);
      setMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
      
      // ROS_INFO("moving backward...");
    }
    //left
    else if(mControlDirection == 4){
      // T5.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust - mControlPowerTemp);
      setMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
      
      // ROS_INFO("moving left...");
    }
    double calculatedDistance = 0;
    double l1 = max(positionX, positionXPrev) - min(positionX, positionXPrev);
    double l2 = max(positionY, positionYPrev) - min(positionY, positionYPrev);
    calculatedDistance = sqrt(l1*l1 + l2*l2);
    // cout << "calculatedDistance: " << calculatedDistance << " mControlDistance: " << mControlDistance << endl;
    // cout << "mControlPower: " << mControlPower << endl;
    
    if(calculatedDistance >= mControlDistance){
      mControlMode2 = false;
      ROS_INFO("Mode 2 finished.\n");
      movementControlFinish();
    }
  }
  else if(mControlMode5){
    //forward
    if(mControlDirection == 1){
      // T6.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust - mControlPowerTemp);
      setMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
      
      //Testing-------------------
      // positionY += 0.05;
      // ROS_INFO("moving forward...");
    }
    //right
    else if(mControlDirection == 2){
      // T5.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust + mControlPowerTemp);
      setMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
      //Testing-------------------
      // positionX += 0.05;
      // ROS_INFO("moving right...");
    }
    //backward
    else if(mControlDirection == 3){
      // T6.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust + mControlPowerTemp);
      setMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
      //Testing-------------------
      // positionY -= 0.05;
      // ROS_INFO("moving backward...");
    }
    //left
    else if(mControlDirection == 4){
      // T5.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust - mControlPowerTemp);
      setMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
      //Testing-------------------
      // positionX -= 0.05;
      // ROS_INFO("moving left...");
    }
    // cout << "loopTime - mControlMode5Timer: " << (loopTime - mControlMode5Timer) << " mControlRunningTime: " << mControlRunningTime << endl;
    if((loopTime - mControlMode5Timer) > (mControlRunningTime*1000)){
      mControlMode5 = false;
      ROS_INFO("Mode 5 finished.\n");
      movementControlFinish();
    }
  }
}

//position keeping control
void positionControl(){
  if(mControlMode1 || mControlMode2 || mControlMode3 || mControlMode4 || mControlMode5){
    // setMHorizontalOffset(0, 0, 0, 0);
    //offset by opposing velocity to keep velocity as close to 0 as possible to fight drift while moving
    if((mControlDirection == 2 || mControlDirection == 4) || keepMovingRight || keepMovingLeft){
      positionControlMOffset -= (velocityY*2/3);
      setMHorizontalOffset(0, positionControlMOffset, 0, -positionControlMOffset);
    }
    else{
      positionControlMOffset -= velocityX;
      setMHorizontalOffset(positionControlMOffset, 0, positionControlMOffset, 0);
    }
  }
  else{
    positionControlMOffset = 0; //rezero position keeping for movementControl
    double pDirection = DVLHelper::getDirection(positionX, positionY, keepPositionX, keepPositionY, yaw);
    double pDistance = DVLHelper::getDistance(positionX, positionY, keepPositionX, keepPositionY);

    double forwardBackwardDistance, rightLeftDistance;
    double forwardBackwardOffset = 0, rightLeftOffset = 0;
    if(pDirection >= 0){
      //on right side of sub rightleftoffset must be positive
      if(pDirection < 90){
        //move forward positive forwardbackwardoffset
        forwardBackwardDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        forwardBackwardOffset = (forwardBackwardDistance/keepPositionThreshold) * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance);

        rightLeftDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        rightLeftOffset = (rightLeftDistance/keepPositionThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance);
      }
      else{
        //move backward negative forwardbackwardoffset
        forwardBackwardDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        forwardBackwardOffset = -((forwardBackwardDistance/keepPositionThreshold) * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance));

        rightLeftDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        rightLeftOffset = (rightLeftDistance/keepPositionThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance);
      }
    }
    else{
      //on left side of sub rightleftoffset must be negative
      if(pDirection > -90){
        //move forward positive forwardbackwardoffset
        forwardBackwardDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        forwardBackwardOffset = (forwardBackwardDistance/keepPositionThreshold) * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance);

        rightLeftDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        rightLeftOffset = -((rightLeftDistance/keepPositionThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance));
      }
      else{
        //move backward negative forwardbackwardoffset
        forwardBackwardDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        forwardBackwardOffset = -(forwardBackwardDistance/keepPositionThreshold * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance));

        rightLeftDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        rightLeftOffset = -((rightLeftDistance/keepPositionThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance));
      }
    }

    //keep prev distance for pid
    forwardBackwardDistancePrev = forwardBackwardDistance;
    rightLeftDistancePrev = rightLeftDistance;

    //cap the offset amount
    if(forwardBackwardOffset > keepPositionMaxPowerForwardBackward) 
      forwardBackwardOffset = keepPositionMaxPowerForwardBackward;
    else if(forwardBackwardOffset < -keepPositionMaxPowerForwardBackward) 
      forwardBackwardOffset = -keepPositionMaxPowerForwardBackward;

    if(rightLeftOffset > keepPositionMaxPowerRightLeft)
      rightLeftOffset = keepPositionMaxPowerRightLeft;
    else if(rightLeftOffset < -keepPositionMaxPowerRightLeft)
      rightLeftOffset = -keepPositionMaxPowerRightLeft;
    //forward positive, right positive offsets
    // cout << "direction: " << pDirection << " distance: " << pDistance << endl;
    // cout << "keep x: " << keepPositionX << " keep y: " << keepPositionY << endl;
    setMHorizontalOffset(rightLeftOffset, forwardBackwardOffset, rightLeftOffset, -forwardBackwardOffset);
  }
}

void setup() {

  //setup motor messages
  mVertical.t1 = base_thrust;
  mVertical.t2 = base_thrust;
  mVertical.t3 = base_thrust;
  mVertical.t4 = base_thrust;
  mHorizontal.t5 = base_thrust;
  mHorizontal.t6 = base_thrust;
  mHorizontal.t7 = base_thrust;
  mHorizontal.t8 = base_thrust;
  //initialize offset for position keeping
  mHorizontalOffset.t5 = 0;
  mHorizontalOffset.t6 = 0;
  mHorizontalOffset.t7 = 0;
  mHorizontalOffset.t8 = 0;
  //Initialize variables
  mControlDirection = 0;
  mControlPower = 0;
  rControlPower = 0;
  hControlPower = 0;
  mControlDistance = 0;
  mControlRunningTime = 0;
  mControlMode5Timer = 0;
  centerTimer = 0;
  rotationTimer = 0;
  rotationTime = 10;
  subIsReady = false;
  isGoingUp = false;
  isGoingDown = false;
  isTurningRight = false;
  isTurningLeft = false;
  keepTurningRight = false;
  keepTurningLeft = false;
  rControlMode3 = false;
  rControlMode4 = false;
  mControlMode1 = false;
  mControlMode2 = false;
  mControlMode3 = false;
  mControlMode4 = false;
  mControlMode5 = false;
  keepMovingForward = false;
  keepMovingRight = false;
  keepMovingBackward = false;
  keepMovingLeft = false;

  //Testing------------------
  feetDepth_read = 0;
  roll = 999;
  pitch = 999;
  yaw = 999;
  positionXPrev = 0;
  positionYPrev = 0;

  positionX = 0;
  positionY = 0;
  positionZ = 0;
  velocityX = 0;
  velocityY = 0;
  velocityZ = 0;

  assignedDepth = topDepth;
  currentDepth.data = feetDepth_read;

//  assignedDepth = 0.2;
  //assignedYaw = -191.5;
  //subIsReady = true;
  // assignedYaw = 64.6;
//  currentRotation.data = yaw;

  hControlStatus.state = 1;
  hControlStatus.depth = 0;
  rControlStatus.state = 1;
  rControlStatus.rotation = 0;

  // MControl
  // state => 0: off, 1: on with power, 2: on with distance, 3: centered with front cam, 4: centered with bottom cam
  // direction => 0: none, 1: forward, 2: right, 3: backward, 4: left
  // power => 0: none, x: x power add to the motors
  // distance => 0: none, x: x units away from the object
  // runningTime => 0: none, x: x seconds for the motors to run
  mControlStatus.state = 0;
  mControlStatus.mDirection = 0;
  mControlStatus.power = 0;
  mControlStatus.distance = 0;
  mControlStatus.runningTime = 0;

  // timeCur = millis();
  loopTime = millis();
  timePrev = loopTime;
  loopTimePrev = loopTime;

  ROS_INFO("Sub is staying. Waiting to receive data from master...\n");
}

//main program loop
void loop() {
  loopTime = millis();  // update time

  // Rotation
  // duty cycle and PWM calculation for orientation
  // dutyCycl_orient = degreeToTurn() / 180.0;
  // PWM_Motors_orient = dutyCycl_orient * 400; 
  
  if(subIsReady){
    // heightControl();
    movementControl();
    // rotationControl();
    // positionControl();
  } else {
    motorsOff();
  }
  if((loopTime-loopTimePrev) > loopInterval) {
    loopTimePrev = loopTime;


    /////////////////////////////////////////////////////////////////////////////////////////////
    // DEBUG
    // assignedYaw = yaw;
    // cout << "assignedYaw: " << assignedYaw << endl;
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    // if(subIsReady)    

    if(subIsReady){
      elapsedTime = (loopTime - timePrev) /1500; //1500; 
      timePrev = loopTime; //update to current time after done with previous time
      heightControl();
      // movementControl();
      rotationControl();
      positionControl();
    } else {
      motorsOff();
    }

    publishMotors();
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "hardware_interface");
  ros::NodeHandle nh;
  
  hControlPublisher = nh.advertise<robosub::HControl>("height_control_status", 100);
  rControlPublisher = nh.advertise<robosub::RControl>("rotation_control_status", 100);
  mControlPublisher = nh.advertise<robosub::MControl>("movement_control_status", 100);
  mVerticalPublisher = nh.advertise<hardware_interface::MotorVertical>("motor_vertical", 1);
  mHorizontalPublisher = nh.advertise<hardware_interface::MotorHorizontal>("motor_horizontal", 1);

  currentDepthSubscriber = nh.subscribe("current_depth", 1, currentDepthCallback);
  hControlSubscriber = nh.subscribe("height_control", 100, hControlCallback);
  rControlSubscriber = nh.subscribe("rotation_control", 100, rControlCallback);
  mControlSubscriber = nh.subscribe("movement_control", 100, mControlCallback);
  rotationSubscriber = nh.subscribe("current_rotation", 1, rotationCallback);
  dvlSubscriber = nh.subscribe("dvl_status", 1, dvlCallback);
  // dvlHeadingSubscriber = nh.subscribe("dvl_heading", 1, dvlHeadingCallback);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  setup();
  while(ros::ok()){
    loop();
  }

  return 0;
}

